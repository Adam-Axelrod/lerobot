"""Results-table harness: scripts/meca500/eval_auto.py and score_eval.py, with no hardware."""

import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.collision import Clearance
from lerobot.robots.meca500.autocollect.disturbance import sample_shift, shift_frame
from lerobot.robots.meca500.autocollect.metrics import path_metrics, sparc
from lerobot.robots.meca500.autocollect.operator_guard import OperatorGuard

pytest.importorskip("trimesh")
pytest.importorskip("scipy")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "meca500"))


# ------------------------------------------------------------------ the simulated camera knock


def test_shift_frame_moves_the_content_and_blacks_the_exposed_edge():
    frame = np.zeros((40, 60, 3), np.uint8)
    frame[10, 20] = 255
    moved = shift_frame(frame, 5, -3)
    assert moved[7, 25, 0] == 255 and moved[10, 20, 0] == 0
    assert moved.shape == frame.shape
    full = np.full((40, 60, 3), 9, np.uint8)
    moved = shift_frame(full, 5, -3)
    assert moved[:, :5].max() == 0 and moved[-3:].max() == 0  # nothing wraps round
    assert moved[0, 5, 0] == 9
    assert shift_frame(frame, 0, 0) is frame
    assert shift_frame(frame, 100, 0).max() == 0


def test_sample_shift_is_repeatable_the_right_size_and_keeps_the_target_lit():
    centre, radius = np.array([220.2, 351.4]), 163.0
    rng = np.random.default_rng(0)
    for seed in range(200):
        angle, r = rng.uniform(0, 2 * np.pi), radius * np.sqrt(rng.uniform())
        target = centre + r * np.array([np.cos(angle), np.sin(angle)])
        shift = sample_shift(seed, 58, target, centre, radius)
        assert shift == sample_shift(seed, 58, target, centre, radius)
        assert all(isinstance(v, int) for v in shift)
        assert abs(np.hypot(*shift) - 58) < 1
        assert np.linalg.norm(target - (centre + shift)) <= radius + 1
    assert sample_shift(1, 0, centre, centre, radius) == (0, 0)
    directions = {sample_shift(s, 58, centre, centre, radius) for s in range(20)}
    assert len(directions) > 10


def test_observation_tap_shifts_every_reader_and_logs_only_while_started():
    import eval_auto

    frame = np.zeros((20, 20, 3), np.uint8)
    frame[5, 5] = 200
    robot = MagicMock()
    robot.get_observation = lambda: {
        "cam": frame.copy(),
        **{f"joint_{i + 1}.pos": float(i) for i in range(6)},
    }
    tap = eval_auto.ObservationTap(robot, "cam")
    assert robot.get_observation()["cam"][5, 5, 0] == 200
    tap.shift = (2, 1)
    assert robot.get_observation()["cam"][6, 7, 0] == 200
    tap.start()
    robot.get_observation()
    robot.get_observation()
    log = tap.stop()
    robot.get_observation()
    assert log["obs_q"].shape == (2, 6) and len(log["obs_t"]) == 2
    np.testing.assert_allclose(log["obs_q"][0], np.arange(6.0))


# ------------------------------------------------------------------ the SpaceMouse guard


def guard_at_height(height_mm, clearance_mm=10.0):
    planner = MagicMock()
    tip = np.eye(4)
    tip[2, 3] = 25.0 + height_mm
    planner.kin.tip.return_value = tip
    planner.calib.surface_z_mm = 25.0
    planner.checker.clearance.return_value = Clearance(clearance_mm, "tool", "illuminator")
    return OperatorGuard(planner)


def test_guard_limits_descent_by_height_and_stops_at_the_floor():
    q = np.zeros(6)
    fast = np.array([3.0, 4.0, -50.0])
    np.testing.assert_allclose(guard_at_height(20.0).filter(q, fast), fast)  # far above: unlimited
    np.testing.assert_allclose(guard_at_height(1.0).filter(q, fast)[2], -5.0)
    np.testing.assert_allclose(guard_at_height(0.001).filter(q, fast)[2], -0.1)  # still arrives
    on_dish = guard_at_height(0.0)
    v = on_dish.filter(q, fast)
    np.testing.assert_allclose(v, [3.0, 4.0, 0.0])  # sideways still allowed
    assert on_dish.floor_hit
    below = guard_at_height(-0.05)
    assert below.filter(q, fast)[2] == 0
    assert below.filter(q, np.array([0.0, 0.0, 2.0]))[2] == 2.0  # can always lift off


def test_guard_never_lets_the_descent_speed_outrun_the_stale_joint_reading():
    """Worst case over a descent at full coarse command: the overshoot past the floor stays tiny."""
    z, dt, latency = 10.0, 0.005, 0.03
    history = [z] * int(latency / dt)
    guard = guard_at_height(0.0)
    tip = np.eye(4)
    guard.planner.kin.tip.return_value = tip
    guard.planner.calib.surface_z_mm = 0.0
    for _ in range(20000):
        tip[2, 3] = history[0]  # the guard only sees the height `latency` ago
        z += guard.filter(np.zeros(6), np.array([0.0, 0.0, -50.0]))[2] * dt
        history = [*history[1:], z]
    assert z > -0.02  # um scale, inside the policy's 0.1 mm press limit
    assert z < 0.01  # and it does reach the dish


def test_guard_trips_on_the_collision_model_and_holds_until_reset():
    assert np.any(
        guard_at_height(10.0, clearance_mm=-0.73).filter(np.zeros(6), np.ones(3))
    )  # within the allowance
    guard = guard_at_height(10.0, clearance_mm=-1.5)
    assert np.all(guard.filter(np.zeros(6), np.array([1.0, 1.0, 1.0])) == 0)
    assert "illuminator" in guard.tripped
    guard.planner.checker.clearance.return_value = Clearance(5.0, "tool", "illuminator")
    assert np.all(guard.filter(np.zeros(6), np.array([1.0, 1.0, 1.0])) == 0)  # still latched
    guard.reset()
    assert guard.tripped is None
    assert np.all(guard.filter(np.zeros(6), np.array([1.0, 1.0, 1.0])) == 1)
    guard.check_clearance = False  # backing out after a trip
    guard.planner.checker.clearance.return_value = Clearance(-5.0, "tool", "illuminator")
    assert np.all(guard.filter(np.zeros(6), np.array([1.0, 1.0, 1.0])) == 1)


def test_spacemouse_translation_only_filter_and_pause():
    from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
    from lerobot.teleoperators.meca500_spacemouse.meca500_spacemouse import Meca500Spacemouse

    teleop = Meca500Spacemouse(
        Meca500SpacemouseConfig(
            id="t", translation_only=True, start_paused=True, alpha=1.0, axis_signs=[1.0] * 6
        )
    )
    teleop.robot = MagicMock()
    teleop.robot.GetRtJointPos.return_value = [0.0] * 6
    teleop._read_twist = lambda: np.ones(6)
    commands = []

    def stop_after_one(*v):
        commands.append(v)
        teleop._running = False

    teleop.robot.MoveLinVelWrf = stop_after_one
    teleop.velocity_filter = lambda q, v: v * [1, 1, 0]
    teleop._running = True
    teleop._guidance_loop()
    assert commands[-1] == (50.0, 50.0, 0.0, 0.0, 0.0, 0.0)

    def boom(q, v):
        raise RuntimeError("no joints")

    teleop.velocity_filter = boom
    teleop._running = True
    teleop._guidance_loop()  # a failing filter holds the arm and does not end the loop by raising
    assert commands[-1][:3] == (0.0, 0.0, 0.0)

    teleop.robot.MoveLinVelWrf = MagicMock()
    teleop._precision_active = True
    teleop.pause()
    assert teleop._homing.is_set() and not teleop.precision_active
    teleop.robot.MoveLinVelWrf.assert_called_with(0, 0, 0, 0, 0, 0)
    teleop.resume()
    assert not teleop._homing.is_set()


# ------------------------------------------------------------------ metrics


def test_sparc_prefers_a_smooth_speed_profile():
    t = np.linspace(0, 1, 201)
    min_jerk = 30 * t**2 * (1 - t) ** 2  # the classic smooth reach; SPARC is about -1.6
    assert sparc(min_jerk, 200) == pytest.approx(-1.6, abs=0.2)
    jerky = min_jerk * (1 + 0.5 * np.sin(2 * np.pi * 8 * t))
    assert sparc(jerky, 200) < sparc(min_jerk, 200) - 0.2
    assert np.isnan(sparc(np.zeros(50), 200))


def test_path_metrics_on_a_straight_reach_then_a_wait():
    t = np.arange(0, 12, 0.05)
    x = np.clip(t, 0, 10) * 2.0  # 20 mm in 10 s, then still for 2 s
    tip = np.column_stack([x, np.zeros_like(x), np.zeros_like(x)])
    m = path_metrics(t, tip)
    assert m["arrival_s"] == pytest.approx(10.0, abs=0.1)
    assert m["economy"] == pytest.approx(1.0, abs=1e-6)
    detour = tip.copy()
    detour[:, 1] = 5 * np.sin(np.pi * np.clip(t, 0, 10) / 10)
    assert path_metrics(t, detour)["economy"] < 0.95


# ------------------------------------------------------------------ the full session, each METHOD


class FakeStage:
    is_homed = True

    def __init__(self):
        self.xy = np.array([50.3, 34.8])

    def position(self):
        return self.xy

    def move_to(self, xy):
        self.xy = np.asarray(xy, float)
        return self.xy

    def disconnect(self):
        pass


class FakeArm:
    """A controller connection: joints follow MoveJoints at once, velocity commands integrate."""

    def __init__(self, q):
        self.q = np.asarray(q, float)
        self.velocities = []

    def GetRtJointPos(self):  # noqa: N802
        return list(self.q)

    GetRtTargetJointPos = GetRtJointPos  # noqa: N815
    GetJoints = GetRtJointPos  # noqa: N815

    def MoveJoints(self, *q):  # noqa: N802
        self.q = np.asarray(q, float)

    def MoveLinVelWrf(self, *v):  # noqa: N802
        self.velocities.append(v)

    def WaitIdle(self, timeout=None):  # noqa: N802
        pass

    def GetStatusRobot(self):  # noqa: N802
        return MagicMock(error_status=False, pause_motion_status=False)

    def __getattr__(self, _name):
        return MagicMock()


class FakeRobot:
    """Just enough of Meca500 for eval_auto: a camera that draws the tip where the arm has it."""

    name = "meca500_basler"
    config = MagicMock(default_joint_vel=25.0)
    is_connected = True
    observation_features = {
        **{f"joint_{i + 1}.pos": float for i in range(6)},
        "microscope_cam": (880, 700, 3),
    }
    action_features = {**{f"joint_{i + 1}.pos": float for i in range(6)}, "precision.state": float}

    def __init__(self, arm, planner, model_error_px=0.0):
        self.robot = arm
        self.planner = planner
        self.model_error_px = model_error_px
        self.truth = None  # the real camera geometry, if the methods use another (a slid robot)
        self.sent = []

    def get_observation(self):
        frame = np.full((880, 700, 3), 180, np.uint8)
        q = self.robot.q
        tip = self.planner.kin.tip(q)[:3, 3]
        if tip[2] - self.planner.calib.surface_z_mm < 6:  # blur hides it higher up
            u, v = (self.truth or self.planner.calib).xy_to_pixel(tip[:2])[0] + self.model_error_px
            axis = np.array([1.0, 0.0])
            for s in np.arange(0, 700, 1.0):
                x, y = int(u + s * axis[0]), int(v + s * axis[1])
                if 0 <= x < 700 and 0 <= y < 880:
                    frame[max(y - 2, 0) : y + 3, x] = 40
        return {**{f"joint_{i + 1}.pos": float(q[i]) for i in range(6)}, "microscope_cam": frame}

    def send_action(self, action):
        self.robot.MoveJoints(*[action[f"joint_{i + 1}.pos"] for i in range(6)])
        self.sent.append(action)
        return action

    def connect(self):
        pass

    def configure(self):
        pass

    def disconnect(self):
        pass


def run_session(monkeypatch, tmp_path, method, shift=0, episodes=2, policy=None):
    """eval_auto.main() on the real bench model with the hardware faked."""
    import eval_auto
    import record_auto

    monkeypatch.chdir(tmp_path)
    bench_real = BenchConfig.load
    monkeypatch.setattr(eval_auto.BenchConfig, "load", lambda rig: bench_real(rig))
    monkeypatch.setattr(eval_auto, "METHOD", method)
    monkeypatch.setattr(eval_auto, "IMAGE_SHIFT_PX", shift)
    monkeypatch.setattr(eval_auto, "NUM_EPISODES", episodes)
    monkeypatch.setattr(eval_auto, "check_focus", lambda scope: 4.0)
    monkeypatch.setattr(eval_auto, "precise_sleep", lambda s: None)
    monkeypatch.setattr(record_auto, "BACKGROUND_WAIT_S", 0.0, raising=False)
    home = np.asarray(eval_auto.require_home_joints(eval_auto.RIG), float)
    arm = FakeArm(home)
    stage = FakeStage()

    def connect_stage(bench, calib):
        from lerobot.robots.meca500.autocollect.dish_map import DishHeightMap
        from lerobot.robots.meca500.autocollect.stage import StageModel

        return (
            stage,
            StageModel.from_bench(bench.section("scope"), calib),
            DishHeightMap.load(bench.dish_map_path()),
        )

    monkeypatch.setattr(eval_auto, "connect_stage", connect_stage)
    holder = {}
    real_planner = eval_auto.build_planner

    def build_planner(bench):
        holder["planner"] = real_planner(bench)
        return holder["planner"]

    monkeypatch.setattr(eval_auto, "build_planner", build_planner)
    monkeypatch.setattr(
        eval_auto,
        "make_robot_from_config",
        lambda cfg: holder.setdefault("robot", FakeRobot(arm, holder["planner"])),
    )
    monkeypatch.setattr(eval_auto, "make_ready", lambda meca: None)
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    monkeypatch.setattr(eval_auto, "init_keyboard_listener", lambda: (None, events))
    monkeypatch.setattr(eval_auto, "is_headless", lambda: True)
    monkeypatch.setattr(eval_auto, "log_say", lambda *a, **k: None)
    monkeypatch.setattr(record_auto.time, "sleep", lambda s: None)
    if policy is not None:
        monkeypatch.setattr(eval_auto, "load_policy", lambda model: policy)
    holder["arm"], holder["events"] = arm, events
    return eval_auto, holder


def read_results(tmp_path):
    runs = sorted((tmp_path / "outputs" / "eval_auto").iterdir())
    assert len(runs) == 1
    rows = [json.loads(line) for line in (runs[0] / "results.jsonl").read_text().splitlines()]
    return runs[0], rows


def test_scripted_session_runs_the_plan_and_saves_a_scoreable_record(monkeypatch, tmp_path):
    eval_auto, holder = run_session(monkeypatch, tmp_path, "scripted")
    eval_auto.main()
    run, rows = read_results(tmp_path)
    assert run.name.startswith("scripted_")
    assert [r["seed"] for r in rows] == [eval_auto.SEED_BASE, eval_auto.SEED_BASE + 1]
    assert rows[0]["method"] == "scripted" and rows[0]["reason"] == "finished"
    # The arm really followed the plan to the target: FK agrees to the calibration's precision.
    assert rows[0]["fk_error_um"] < 40
    assert (run / "ep000" / "final_overlay.png").exists()
    with np.load(run / "ep000" / "traj.npz") as z:
        assert len(z["obs_t"]) > 50 and z["obs_q"].shape[1] == 6
    assert abs(rows[0]["tip_height_um"]) < 20


def test_the_seeds_fix_the_test_set_across_methods_and_shift(monkeypatch, tmp_path):
    (tmp_path / "a").mkdir()
    eval_auto, _ = run_session(monkeypatch, tmp_path / "a", "scripted")
    eval_auto.main()
    _, plain = read_results(tmp_path / "a")
    monkeypatch.undo()
    (tmp_path / "b").mkdir()
    eval_auto, _ = run_session(monkeypatch, tmp_path / "b", "scripted", shift=58)
    eval_auto.main()
    run, knocked = read_results(tmp_path / "b")
    assert run.name.startswith("scripted_shift58_")
    for a, b in zip(plain, knocked, strict=True):
        assert a["target_px"] == b["target_px"] and a["stage_xy_mm"] == b["stage_xy_mm"]
        assert abs(np.hypot(*b["image_shift_px"]) - 58) < 1
        # Nothing physical moved, so the arm went to the same place. In the shifted image, the
        # open-loop controller therefore misses by about the shift, and FK + shift says so.
        assert b["fk_error_um"] == pytest.approx(58 * 12.85, abs=60)


@pytest.mark.parametrize(
    "method, shift_px, corrected",
    [("expert", 16, True), ("expert", 58, False), ("scripted", 16, False)],
)
def test_who_copes_with_a_knock_measured_end_to_end_by_the_detector(
    monkeypatch, tmp_path, method, shift_px, corrected
):
    """Through the real detector and corrections: a shift inside the expert's 0.5 mm cap is corrected,
    one beyond it is skipped, and the open-loop controller misses by the whole shift."""
    eval_auto, _ = run_session(monkeypatch, tmp_path, method, shift=shift_px, episodes=1)
    eval_auto.main()
    _, rows = read_results(tmp_path)
    row = rows[0]
    assert row["vision_score"] > 0.15
    expected = 40 if corrected else shift_px * 12.85
    assert row["vision_error_um"] == pytest.approx(expected, abs=60)
    # With the shift added, forward kinematics agrees with the detector about the same frame.
    assert row["fk_error_um"] == pytest.approx(row["vision_error_um"], abs=60)
    if method == "expert":
        notes = [c["note"] for c in row["corrections"]]
        assert any("too large" in n for n in notes) != corrected


def test_expert_counts_an_abort_as_a_failure(monkeypatch):
    import eval_auto

    def aborted(*a, **k):
        raise eval_auto.EpisodeAbortedError("would collide")

    monkeypatch.setattr(eval_auto, "run_corrected_episode", aborted)
    run = eval_auto.run_expert(
        None, None, None, None, "k", None, {"exit_early": True, "rerecord_episode": True}, None, None
    )
    assert run == {"reason": "stopped: would collide"}


def test_policy_session_measures_what_the_policy_did(monkeypatch, tmp_path):
    import torch

    class Still:
        def reset(self):
            pass

    eval_auto, _ = run_session(
        monkeypatch, tmp_path, "policy", episodes=1, policy=(Still(), Still(), Still())
    )
    # A precise action that does not move: it settles after SETTLED_S and is still measured.
    monkeypatch.setattr(eval_auto, "predict_action", lambda *a, **k: torch.tensor([[0, 0, 0, 0, 0, 0, 1.0]]))
    eval_auto.main()
    run, rows = read_results(tmp_path)
    assert rows[0]["reason"] == "settled" and rows[0]["method"] == "policy"
    assert rows[0]["clamped_steps"] == 0
    with np.load(run / "ep000" / "traj.npz") as z:
        assert "obs_q" in z and "delta" in z


def test_human_session_starts_on_the_key_ends_on_the_key_and_is_measured(monkeypatch, tmp_path):
    eval_auto, holder = run_session(monkeypatch, tmp_path, "human", episodes=1)
    arm, events = holder["arm"], holder["events"]

    class FakeTeleop:
        """The real class needs a SpaceMouse; this keeps its pause/resume/precision contract."""

        def __init__(self, cfg):
            assert cfg.translation_only and cfg.start_paused and not cfg.home_hotkey
            self.robot = arm
            self.velocity_filter = None
            self.paused = True
            self.precision_active = False

        def connect(self):
            pass

        def disconnect(self):
            pass

        def pause(self):
            self.paused = True

        def resume(self):
            self.paused = False

    class FakeView:
        closed = False

        def __init__(self, title):
            self.titles = []

        def show(self, frame, title=""):
            self.titles.append(title)
            holder["view"] = self
            events["exit_early"] = True  # the operator presses the right arrow at once

        def close(self):
            pass

    monkeypatch.setattr(eval_auto, "Meca500Spacemouse", FakeTeleop)
    monkeypatch.setitem(sys.modules, "live_view", MagicMock(LiveView=FakeView))
    eval_auto.main()
    run, rows = read_results(tmp_path)
    assert run.name.startswith("human_experienced")
    assert rows[0]["method"] == "human" and rows[0]["operator"] == "experienced"
    assert rows[0]["reason"] == "operator"
    assert "press the right arrow to start" in holder["view"].titles[0]
    assert holder["robot"].sent == []  # nothing went through the read-only connection


class FakeAsrDetector:
    """Sees the tip where forward kinematics puts it, once it is low and inside the lit field."""

    def __init__(self, planner, robot, centre, radius, noise_px=0.5):
        self.planner, self.robot = planner, robot
        self.centre, self.radius = np.asarray(centre, float), float(radius)
        self.rng = np.random.default_rng(0)
        self.noise = noise_px
        self.calls = 0

    def detect(self, rgb):
        from lerobot.robots.meca500.autocollect.asr_bridge import TipSighting

        self.calls += 1
        tip = self.planner.kin.tip(self.robot.robot.q)[:3, 3]
        px = self.planner.calib.xy_to_pixel(tip[:2])[0]
        if tip[2] - self.planner.calib.surface_z_mm > 6 or np.linalg.norm(px - self.centre) > self.radius:
            return None
        px = px + self.rng.normal(0, self.noise, 2)
        return TipSighting((float(px[0]), float(px[1])), 0.97, __import__("time").perf_counter())


class FakeAsrService:
    """ASR's service contract over the real host adapter: a plain proportional servo on the
    detector's pixel, every move through Meca500AsrHost.execute_tip_delta."""

    def __init__(self, host, detector_holder, converge_px):
        self.host, self.holder, self.converge_px = host, detector_holder, converge_px
        self.runtime = object()

    def initialize(self, timeout=None):
        return MagicMock(success=True)

    def capture_anchor(self, request, timeout=None):
        s = self.holder["detector"].detect(None)
        return MagicMock(valid=s is not None, reason="ok", visual_tip_px=None if s is None else s.px)

    def submit_task(self, goal):
        from action_state_runtime.contracts import CancelToken, TipMotionCommand

        planner = self.host.planner
        target = np.asarray(goal.target_px, float)
        converged, steps, dist = False, 0, None
        for steps in range(1, 9):
            s = self.holder["detector"].detect(None)
            dist = float(np.linalg.norm(target - s.px))
            if dist <= self.converge_px:
                converged = True
                break
            d_mm = planner.calib.pixel_to_xy(target)[0] - planner.calib.pixel_to_xy(s.px)[0]
            cmd = TipMotionCommand("g", steps, (d_mm[0] / 1000, d_mm[1] / 1000, 0.0), 1, "positioning")
            assert self.host.execute_tip_delta(cmd, CancelToken()).success
        result = MagicMock(
            converged=converged, reason="converged" if converged else "max_steps", servo_steps=steps,
            final_tip_px=s.px, final_distance_px=dist,
        )
        return MagicMock(accepted=True, result=lambda timeout=None: result)


def run_asr_session(monkeypatch, tmp_path, *, offset=(0.0, 0.0), sees=True, method="policy_asr"):
    """policy_asr with a policy that replays each episode's planned joint path as deltas."""
    import torch

    class Still:
        def reset(self):
            pass

        config = MagicMock(n_action_steps=1)

    eval_auto, holder = run_session(monkeypatch, tmp_path, method, episodes=1, policy=(Still(), Still(), Still()))
    monkeypatch.setattr(eval_auto, "BASE_OFFSET_MM", offset)
    monkeypatch.setattr(eval_auto, "EPISODE_TIME_S", 600.0)  # a loaded test machine runs the replay slowly
    # The fake detector reports the free end itself: no ASR tip offset in this world.
    (tmp_path / "asr_tip_offset.json").write_text(json.dumps({"offset_px": [0.0, 0.0], "n": 0}))
    monkeypatch.setattr(eval_auto, "ASR_TIP_OFFSET_FILE", str(tmp_path / "asr_tip_offset.json"))
    monkeypatch.setattr(eval_auto, "SAVE_LIVE_FRAMES", False)
    real_plan_episode = eval_auto.StagePlanner.plan_episode

    def plan_episode(self, seed):
        xy, plan = real_plan_episode(self, seed)
        holder["deltas"] = list(np.diff(plan.joints, axis=0))
        holder["flags"] = list(plan.precision[1:])
        holder["plan"] = plan
        return xy, plan

    monkeypatch.setattr(eval_auto.StagePlanner, "plan_episode", plan_episode)
    holder["seen_states"] = []

    def predict_action(frame, *a, **k):
        __import__("time").sleep(0.01)  # precise_sleep is off: give the watcher thread real time
        if holder.get("detector") is not None and holder["detector"].detect(None) is not None:
            __import__("time").sleep(0.3)  # the tip is in view: a real 20 Hz loop leaves the watcher time
        holder["seen_states"].append(np.asarray(frame["observation.state"][:6], float))
        if not holder["deltas"]:
            return torch.tensor([[0, 0, 0, 0, 0, 0, 1.0]])
        return torch.tensor([[*holder["deltas"].pop(0), holder["flags"].pop(0)]], dtype=torch.float32)

    monkeypatch.setattr(eval_auto, "predict_action", predict_action)

    def make_service(host, site, out, converge_px):
        return FakeAsrService(host, holder, converge_px)

    def make_detector(runtime, centre, radius):
        det = FakeAsrDetector(holder["planner"], holder["robot"], centre, radius)
        if not sees:
            det.detect = lambda rgb: None
        holder["detector"] = det
        return det

    monkeypatch.setattr(eval_auto, "make_service", make_service)
    monkeypatch.setattr(eval_auto, "AsrTipDetector", make_detector)
    eval_auto.main()
    return eval_auto, holder


def test_policy_asr_hands_over_when_the_tip_appears_and_servos_onto_the_target(monkeypatch, tmp_path):
    eval_auto, holder = run_asr_session(monkeypatch, tmp_path)
    run, rows = read_results(tmp_path)
    row = rows[0]
    assert run.name.startswith(f"{eval_auto.EVAL_NAME}_asr_")
    assert row["method"] == "policy_asr" and row["reason"] == "finished", row
    assert [s["stage"] for s in row["stages"]] == ["approach", "handover", "lower", "servo", "touch"]
    assert row["stages"][0]["reason"] == "handover: tip in view"
    assert not row["search_used"] and row["failure_stage"] is None
    assert row["servo"]["converged"]
    assert row["fk_error_um"] < 60 and abs(row["tip_height_um"]) < 20
    assert row["time_to_view_s"] >= 0 and row["handover"]["tip_height_um"] > 0
    assert (run / "ep000" / "episode.mp4").stat().st_size > 0
    # The paper figure from this trial: four frames + tip speed, one column wide, small.
    import trial_figure

    monkeypatch.setattr(sys, "argv", ["trial_figure.py", str(run), "--episode", "0", "--out", str(tmp_path / "fig.pdf")])
    trial_figure.main()
    assert 5_000 < (tmp_path / "fig.pdf").stat().st_size < 300_000
    holder["fig"] = tmp_path / "fig.pdf"


def test_policy_asr_searches_then_reports_where_it_failed(monkeypatch, tmp_path):
    eval_auto, holder = run_asr_session(monkeypatch, tmp_path, sees=False)
    _, rows = read_results(tmp_path)
    row = rows[0]
    assert row["search_used"] and row["failure_stage"] == "search"
    search = next(s for s in row["stages"] if s["stage"] == "search")
    assert search["found"] is False and search["points_tried"] == len(eval_auto.spiral_offsets(1.0, 4.0))


def test_virtual_slide_moves_the_real_tip_and_shows_the_policy_the_planned_joints(monkeypatch, tmp_path):
    eval_auto, holder = run_asr_session(monkeypatch, tmp_path, offset=(2.0, 0.0))
    run, rows = read_results(tmp_path)
    assert "_slide2x0_" in run.name and rows[0]["base_offset_mm"] == [2.0, 0.0]
    assert holder["plan"].recovery is None  # the test set, not whatever record_auto collects
    # The policy's first view is the planned start, while the real arm started 2 mm along x.
    plan, kin = holder["plan"], holder["planner"].kin
    np.testing.assert_allclose(holder["seen_states"][0], plan.start_joints, atol=1e-3)
    with np.load(run / "ep000" / "traj.npz") as z:
        real_start = z["q_cmd"][0]
    shift = kin.tip(real_start)[:3, 3] - kin.tip(plan.joints[1])[:3, 3]
    np.testing.assert_allclose(shift, [2.0, 0.0, 0.0], atol=1e-3)
    # The replayed plan now lands 2 mm off; the servo, working in the real image, brings it back.
    assert rows[0]["reason"] == "finished" and rows[0]["fk_error_um"] < 60
    assert rows[0]["servo"]["converged"] and len(rows[0]["servo"]["moves"]) >= 2
    assert rows[0]["handover"]["vision_to_target_um"] > 1000


def test_scripted_asr_baseline_runs_the_plan_to_pretouch_then_the_same_finish(monkeypatch, tmp_path):
    eval_auto, holder = run_asr_session(monkeypatch, tmp_path, method="scripted_asr")
    run, rows = read_results(tmp_path)
    row = rows[0]
    assert run.name.startswith("scripted_asr_") and row["method"] == "scripted_asr"
    # The pre-touch point can leave the tip outside the lit field: then the search runs first.
    stages = [s["stage"] for s in row["stages"]]
    assert [s for s in stages if s != "search"] == ["approach", "handover", "lower", "servo", "touch"]
    assert row["search_used"] == ("search" in stages)
    assert holder["seen_states"] == []  # no policy was asked anything
    # aimed at the true target, not a decoy (the scripted methods have no image correction)
    assert getattr(holder["plan"], "decoy_px", None) is None or np.allclose(holder["plan"].decoy_px, holder["plan"].target_px, atol=0.01)
    assert row["reason"] == "finished" and row["fk_error_um"] < 60


def test_apply_slide_measures_the_slide_from_the_calibrations_and_moves_the_scene(monkeypatch, tmp_path):
    import shutil

    import apply_slide

    real = BenchConfig.load("meca500_basler")
    shutil.copy(real.calibration_path(), tmp_path / real.calibration_path().name)
    calib_after = json.loads(real.calibration_path().read_text())
    before = dict(calib_after)
    before["px_to_xy"] = [row[:] for row in calib_after["px_to_xy"]]
    before["px_to_xy"][0][2] += 2.0  # before the slide, every pixel sat 2 mm further along robot +x:
    before["px_to_xy"][1][2] -= 1.0  # the base has since moved +2, -1 mm
    (tmp_path / "before.json").write_text(json.dumps(before))

    bench = BenchConfig.load("meca500_basler")
    object.__setattr__(bench, "root", tmp_path)
    monkeypatch.setattr(apply_slide.BenchConfig, "load", lambda rig: bench)
    monkeypatch.setattr(sys, "argv", ["apply_slide.py", "--before", str(tmp_path / "before.json")])
    apply_slide.main()
    m = json.loads((tmp_path / "slide_measurement.json").read_text())
    np.testing.assert_allclose(m["slide_mm"], [2.0, -1.0], atol=1e-6)
    assert abs(m["rotation_deg"]) < 1e-6
    t = bench.registration_transform()
    np.testing.assert_allclose(t[:3, 3], [-2.0, 1.0, 0.0], atol=1e-9)  # the scene moved the other way
    monkeypatch.setattr(sys, "argv", ["apply_slide.py", "--reset"])
    apply_slide.main()
    assert bench.registration_transform() is None


def slid_bench(tmp_path, slide_mm=(1.0, 0.0)):
    """A copy of the real bench after a physical slide of ``slide_mm``: the normal files are the
    re-measured (true) ones; the pinned calibration is from before the slide, and its dish height
    is 40 um stale (drift since), which the methods must NOT use."""
    import shutil

    from lerobot.robots.meca500.autocollect.slide import MEASUREMENT, PINNED_CALIBRATION

    real = BenchConfig.load("meca500_basler")
    root = tmp_path / "benches" / "meca500_basler"
    shutil.copytree(real.root, root, ignore=shutil.ignore_patterns("*.bak.json", "preview.*", "beads", "*.bad-*"))
    d = np.asarray(slide_mm, float)
    before = json.loads(real.calibration_path().read_text())
    before["px_to_xy"][0][2] += d[0]  # before the slide, each pixel sat d further along the robot axes
    before["px_to_xy"][1][2] += d[1]
    before["surface_z_mm"] += 0.040
    (root / PINNED_CALIBRATION).write_text(json.dumps(before))
    t = np.eye(4)
    t[:2, 3] = -d
    (root / "registration.json").write_text(json.dumps({"t_robot_cad": t.tolist(), "apply": True}))
    (root / MEASUREMENT).write_text(json.dumps({"slide_mm": d.tolist(), "rotation_deg": 0.0, "surface_z_change_um": 0.0}))
    return root


def test_split_benches_pins_the_methods_to_the_pre_slide_calibration(tmp_path):
    from lerobot.robots.meca500.autocollect.slide import PINNED_CALIBRATION, split_benches

    real = BenchConfig.load("meca500_basler")
    assert split_benches(real).method is real  # the original position: one bench
    root = slid_bench(tmp_path)
    b = split_benches(BenchConfig.load("meca500_basler", benches_dir=root.parent))
    centre = np.array([213.0, 319.5])
    shift = b.method.calibration().pixel_to_xy(centre)[0] - b.truth.calibration().pixel_to_xy(centre)[0]
    np.testing.assert_allclose(shift, [1.0, 0.0], atol=1e-9)
    # in-plane map pinned, height current
    assert b.method.calibration().surface_z_mm == b.truth.calibration().surface_z_mm
    assert b.method.dish_map_path() == b.truth.dish_map_path()
    assert b.method.registration_transform() is not None  # the methods' collision model still moved
    (root / PINNED_CALIBRATION).unlink()  # half a slide state is refused, not guessed at
    with pytest.raises(RuntimeError):
        split_benches(BenchConfig.load("meca500_basler", benches_dir=root.parent))


def test_after_a_physical_slide_scripted_aims_with_the_old_calibration_and_is_measured_with_the_new(monkeypatch, tmp_path):
    """The point of the slide test: scripted is not recalibrated, so it misses by the slide - and
    the final measurement, on the true calibration, says so."""
    import eval_auto

    root = slid_bench(tmp_path, (1.0, 0.0))
    (tmp_path / "run").mkdir()
    load = BenchConfig.load  # the real loader, before run_session replaces it on the class
    eval_auto_mod, holder = run_session(monkeypatch, tmp_path / "run", "scripted", episodes=1)
    monkeypatch.setattr(eval_auto.BenchConfig, "load", lambda rig, **kw: load(rig, benches_dir=root.parent))
    real_make = eval_auto.make_robot_from_config

    def make_robot(cfg):
        robot = real_make(cfg)
        robot.truth = load("meca500_basler", benches_dir=root.parent).calibration()
        return robot

    monkeypatch.setattr(eval_auto, "make_robot_from_config", make_robot)
    eval_auto.main()
    _, rows = read_results(tmp_path / "run")
    row = rows[0]
    assert row["slide"]["slide_mm"] == [1.0, 0.0]
    assert row["fk_error_um"] == pytest.approx(1000, abs=60)
    assert row["vision_error_um"] == pytest.approx(1000, abs=80)
    assert abs(row["tip_height_um"]) < 20  # touched the current dish, not the stale pinned height


def test_virtual_slide_round_trips():
    import record_auto

    from lerobot.robots.meca500.autocollect.asr_bridge import VirtualSlide

    planner = record_auto.build_planner(BenchConfig.load("meca500_basler"))
    q = planner.sample_episode(5_000_000).start_joints
    slide = VirtualSlide(planner.kin, (2.0, -4.0))
    real = slide.to_real(q)
    np.testing.assert_allclose(planner.kin.tip(real)[:3, 3] - planner.kin.tip(q)[:3, 3], [2, -4, 0], atol=1e-3)
    np.testing.assert_allclose(planner.kin.tip(real)[:3, :3], planner.kin.tip(q)[:3, :3], atol=1e-5)
    np.testing.assert_allclose(slide.to_virtual(real), q, atol=1e-3)


def test_tip_watcher_needs_consecutive_confident_sightings():
    import time

    from lerobot.robots.meca500.autocollect.asr_bridge import TipSighting, TipWatcher

    seq = iter([None, (10, 10, 0.95), (300, 300, 0.6), (50, 50, 0.9), (55, 52, 0.92)] + [None] * 1000)

    class Det:
        def detect(self, rgb):
            s = next(seq)
            return None if s is None else TipSighting((s[0], s[1]), s[2], time.perf_counter())

    w = TipWatcher(Det(), lambda: np.zeros((4, 4, 3), np.uint8), min_confidence=0.8, hits=2, max_jump_px=40)
    w.start()
    assert w.seen.wait(2.0)
    w.stop()
    assert w.sighting.px == (55, 52) and len(w.history) == 5


def test_human_start_keeps_the_position_and_takes_the_touch_orientation():
    import eval_auto
    import record_auto

    planner = record_auto.build_planner(BenchConfig.load("meca500_basler"))
    plan = planner.sample_episode(5_000_000)
    q = eval_auto.human_start_joints(planner, plan)
    tip, start, touch = (
        planner.kin.tip(q),
        planner.kin.tip(plan.start_joints),
        planner.kin.tip(plan.joints[-1]),
    )
    np.testing.assert_allclose(tip[:3, 3], start[:3, 3], atol=0.01)
    np.testing.assert_allclose(tip[:3, :3], touch[:3, :3], atol=1e-3)
    assert planner.checker.is_clear(q)


# ------------------------------------------------------------------ scoring


def make_run(root: Path, name: str, rows: list[dict], verdicts=None):
    run = root / name
    run.mkdir()
    (run / "results.jsonl").write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    if verdicts:
        (run / "verdicts.json").write_text(json.dumps(verdicts))
    return run


def base_row(ep, error, reason="settled", target=(100.0, 200.0)):
    return {
        "episode": ep,
        "seed": 5_000_000 + ep,
        "target_px": list(target),
        "reason": reason,
        "vision_error_um": error,
    }


def test_scoring_applies_success_stops_and_by_eye_verdicts(tmp_path):
    import score_eval

    run = make_run(
        tmp_path,
        "act_20261105-140000",
        [
            base_row(0, 20.0),
            base_row(1, 210.0),  # the detector's false reading: a person saw it in the ring
            base_row(2, 30.0, reason="stopped: tool within margin of illuminator (-1.0 mm)"),
            base_row(3, None),  # not detected, no verdict
            base_row(4, 40.0),
            base_row(5, 45.0),
            base_row(6, 30.0, reason="stopped: commanded tip -120 um below the dish surface"),
        ],
        verdicts={"1": 35.0, "5": "miss"},
    )
    rows = score_eval.load_run(run)
    trials = [score_eval.score_trial(r, r["_verdict"], None) for r in rows]
    assert [t["success"] for t in trials] == [True, True, False, False, True, False, True]
    s = score_eval.summarise(trials)
    assert (s["n"], s["successes"], s["collisions"], s["press_stops"], s["no_distance"]) == (7, 4, 1, 1, 1)
    assert s["distance_median"] == pytest.approx(32.5)
    assert "4/7" in score_eval.format_row("act", s)


def test_check_frames_draws_the_scale_round_the_target(tmp_path):
    import check_frames
    import cv2

    frame = np.full((440, 700, 3), 180, np.uint8)
    out = check_frames.annotate(frame, (350, 220), 12.85, tip=(380, 220), label="ep 0")
    size = check_frames.OUT_SIZE
    assert out.shape == (size, size, 3)
    centre = size // 2
    radius = round(600 / 12.85 * size / (2 * check_frames.MIN_HALF_WINDOW_PX))
    band = out[centre - 2 : centre + 3, centre + radius - 2 : centre + radius + 3]
    assert band[..., 0].min() < 120 and band[..., 1].max() > 200  # yellow in BGR, on a grey frame
    # A target near the edge pads with black rather than shifting the circles off the target.
    assert check_frames.half_window((350, 220), (650, 220)) == 340  # a far tip stays in view
    assert check_frames.half_window((350, 220), None) == check_frames.WIDE_HALF_WINDOW_PX
    edge = check_frames.annotate(frame, (5, 5), 12.85)
    assert edge.shape == out.shape and edge[40, 40].max() == 0

    run = tmp_path / "run"
    (run / "ep000").mkdir(parents=True)
    cv2.imwrite(str(run / "ep000" / "final.png"), frame)
    row = {"episode": 0, "target_px": [350.0, 220.0], "reason": "time limit"}
    (run / "results.jsonl").write_text(json.dumps(row) + "\n")
    assert "not detected" in check_frames.trial_label(row)
    sheet = check_frames.contact_sheet([out, out, out], columns=2)
    assert sheet.shape == (2 * size, 2 * size, 3)


def test_scoring_reports_policy_asr_with_and_without_the_search(tmp_path):
    import score_eval

    rows = [
        base_row(0, 20.0, reason="finished") | {"search_used": False, "failure_stage": None},
        base_row(1, 30.0, reason="finished") | {"search_used": True, "failure_stage": None},
        base_row(2, None, reason="search did not find the tip") | {"search_used": True, "failure_stage": "search"},
    ]
    run = make_run(tmp_path, "x_asr_20261107-100000", rows)
    trials = [score_eval.score_trial(r, r["_verdict"], None) for r in score_eval.load_run(run)]
    assert score_eval.summarise(trials)["successes"] == 2
    without = [score_eval.without_search(t) for t in trials]
    assert score_eval.summarise(without)["successes"] == 1
    assert [t["failure_stage"] for t in without] == [None, "view", "view"]


def test_scoring_prints_latex_rows_and_keeps_slides_apart(tmp_path, monkeypatch, capsys):
    import score_eval

    def row(ep, err, search=False, fail=None, slide=None, view=True):
        stages = [{"stage": "approach"}] + ([{"stage": "handover"}] if view else [])
        return base_row(ep, err, reason="finished" if fail is None else "search did not find the tip") | {
            "search_used": search, "failure_stage": fail, "stages": stages,
            **({"slide": {"slide_norm_mm": slide}} if slide else {}),
        }

    a = make_run(tmp_path, "m_asr_20261108-100000", [row(0, 20.0), row(1, 80.0, search=True), row(2, None, search=True, fail="search", view=False)])
    b = make_run(tmp_path, "m_asr_20261108-140000", [row(0, 30.0, slide=2.03), row(1, 300.0, slide=2.03)])
    monkeypatch.setattr(sys, "argv", ["score_eval.py", str(a), str(b)])
    score_eval.main()
    out = capsys.readouterr().out
    main_rows = [line for line in out.splitlines() if line.startswith("m_asr") and "&" in line]
    # 0 mm and the 2 mm slide are separate rows, though the session name is the same
    assert main_rows[0].startswith("m_asr & 67 & 0 & 33 & 67 & 50 [35, 65] &")
    assert main_rows[0].rstrip().split("  %")[0].endswith("\\\\")
    assert main_rows[1].startswith("m_asr without search & 33 & 0 & 33 & 33 & 20")
    assert any(r.startswith("m_asr @ 2.0 mm & 100 & 0 & 50 & 50") for r in main_rows)
    moved = [line for line in out.splitlines() if line.startswith("m_asr & 33 & 50 & 50 & 165")]
    assert len(moved) == 1  # success % and median at 0 mm, then at 2 mm
    assert "failed at {'approach': 0, 'search': 1, 'lower': 0, 'servo': 0, 'touch': 0}" in out
    assert "without search {'approach': 2, 'search': 0" in out


def test_scoring_reads_a_run_from_before_the_observation_tap(tmp_path):
    import score_eval

    run = make_run(tmp_path, "act_20261105-140000", [base_row(0, 20.0)])
    (run / "ep000").mkdir()
    np.savez(run / "ep000" / "traj.npz", t=np.arange(5.0), q_meas=np.zeros((5, 6)))
    t, q = score_eval.load_run(run)[0]["_path"]
    assert len(t) == 5 and q.shape == (5, 6)


def test_scoring_refuses_runs_with_different_targets_and_joins_a_continued_session(tmp_path):
    import score_eval

    a = make_run(tmp_path, "act_20261105-140000", [base_row(0, 20.0)])
    b = make_run(tmp_path, "scripted_20261105-150000", [base_row(0, 90.0, target=(100.0, 240.0))])
    c = make_run(tmp_path, "act_20261105-160000", [base_row(1, 20.0)])
    assert score_eval.session_of(a) == score_eval.session_of(c) == "act"
    sessions = {"act": score_eval.load_run(a), "scripted": score_eval.load_run(b)}
    problems = score_eval.check_same_targets(sessions)
    assert len(problems) == 1 and "5000000" in problems[0]


def test_dataset_row_counts_every_saved_episode(tmp_path):
    import pandas as pd
    import score_eval

    root = tmp_path / "ds"
    (root / "data" / "chunk-000").mkdir(parents=True)
    rows = [{"episode_index": i, "final_error_um": e} for i, e in enumerate([300.0, 20.0, 30.0])]
    (root / "autocollect_episodes.jsonl").write_text("\n".join(json.dumps(r) for r in rows))
    frames = []
    q = np.linspace(np.zeros(7), np.ones(7), 40)
    for ep in range(3):
        frames += [{"episode_index": ep, "action": list(v)} for v in q]
    pd.DataFrame(frames).to_parquet(root / "data" / "chunk-000" / "file-000.parquet")
    kin = BenchConfig.load("meca500_basler").kinematics()
    everything = score_eval.summarise(score_eval.dataset_trials(root, 0, kin))
    assert (everything["n"], everything["successes"]) == (3, 2)
    after_fix = score_eval.summarise(score_eval.dataset_trials(root, 1, kin))
    assert (after_fix["n"], after_fix["successes"]) == (2, 2)
    assert np.isfinite(after_fix["sparc_median"])
    checked = score_eval.summarise(score_eval.dataset_trials(root, 0, kin, {"0": "hit"}))
    assert (checked["n"], checked["successes"]) == (3, 3)
