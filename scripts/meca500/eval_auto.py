"""Run one row of the paper's results table on the bench, under record_auto's conditions.

Each episode is set up exactly as record_auto set up a demonstration - same start-pose box,
stage positions and target region, from SEED_BASE seeds the training set never used. The seeds
fix the test set: every METHOD faces the same targets, starts and stage positions.

    1. plan  - stage position, start pose and target pixel;
    2. transit (collision-checked, scripted) to the start pose, then the stage moves;
    3. drive - the marker appears and METHOD drives the arm:
               policy   - MODEL at the recording rate. Every command is checked BEFORE it is
                          sent: per-step clamp, collision clearance of the commanded pose against
                          the bench model, and the tip height above the dish height map (no
                          pressing into the dish). A failed check ends the episode where it stands;
               scripted - the planned path through the calibration, with no image feedback;
               expert   - that path with record_auto's image corrections, which is what collected
                          the training data. An aborted episode counts as a failure here;
               human    - a person on the SpaceMouse, translation only (OperatorGuard: a dish floor
                          and the same collision check). They lower the tip until the floor stops
                          it, then press the right arrow. Starts at the planned approach
                          orientation, since they cannot rotate.
    4. measure - final tip position from forward kinematics AND from the tip detector on the
               raw frame, both against the target, in px and um; raw frames are saved for the
               by-eye check, plus the joint trajectory for the smoothness metric.

The robot runs with relative_actions=False throughout: this script integrates the policy's
delta onto the controller's current target itself, so that it can check the resulting pose
before sending it. Transits are absolute scripted moves, as in record_auto.

IMAGE_SHIFT_PX > 0 simulates a knocked camera (autocollect/disturbance.py): every frame every
reader sees is shifted, in a seeded direction per episode. The final position by forward
kinematics has the shift added, so it stays a valid cross-check of the detector.

Results go to outputs/eval_auto/<session name>_<timestamp>/ - results.jsonl (one line per
episode), epNNN/{background,final,final_overlay}.png and epNNN/traj.npz. score_eval.py turns
them into a table row.
Keys: right arrow ends an episode early (it is still measured; for "human" it also starts each
trial), Esc stops the session. Enter toggles precision mode for "human".
"""

import contextlib
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np
import record_auto
import torch
from record_auto import (
    SETTLE_TIMEOUT_S,
    EpisodeAbortedError,
    StagePlanner,
    build_planner,
    connect_stage,
    go_home,
    grab,
    make_ready,
    run_corrected_episode,
    run_plan,
    save_raw,
    settled_background,
    wait_settled,
)
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.processor import (
    RobotProcessorPipeline,
    make_default_processors,
    observation_to_transition,
    transition_to_observation,
)
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.disturbance import sample_shift, shift_frame
from lerobot.robots.meca500.autocollect.focus_check import FocusMismatchError, check_focus
from lerobot.robots.meca500.autocollect.operator_guard import OperatorGuard
from lerobot.robots.meca500.autocollect.overlay import TargetOverlayStep, draw_target
from lerobot.robots.meca500.autocollect.planner import PlanningError
from lerobot.robots.meca500.autocollect.tip_actions import camera_axes, tip_action_to_joints
from lerobot.robots.meca500.autocollect.tip_detect import detect_tip
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.meca500_spacemouse.meca500_spacemouse import Meca500Spacemouse
from lerobot.utils.constants import OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener, is_headless
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
# Which row this session fills: "policy", "scripted", "expert" or "human" (see the docstring).
METHOD = "policy"
# Pixels to shift every frame by (12.85 um/px), simulating a knocked camera. 0 is a normal run.
# 16 px (~0.2 mm) is inside the expert's 0.5 mm correction cap; 58 px (~0.75 mm) is beyond it.
IMAGE_SHIFT_PX = 0
# A later session carrying on after a stop: runs seeds SEED_BASE + FIRST_EPISODE onwards.
FIRST_EPISODE = 0
# A local pretrained_model folder or a Hub model repo (downloaded once into the HF cache).
# Must be trained on a DELTA dataset recorded on this rig with the digital target.
# Local copies come from CREATE by scp: the lab PC downloads from the Hub at ~100 KB/s.
MODEL = "outputs/train/basler_stage_touch_decoy_2026-11_delta/checkpoints/100000/pretrained_model"
EVAL_NAME = "basler_stage_touch_decoy_2026-11_delta_act"  # names the "policy" results folder; see session_name()
TASK = "move_pipette_to_target"
NUM_EPISODES = 20
# Training episodes used seeds 1_000_000 + episode index: this range shares none of them.
SEED_BASE = 5_000_000
# Longest demonstration in basler_stage_touch_dataset_2026-11 is 36.9 s (median 21.8 s).
EPISODE_TIME_S = 45.0

# ACT only. None runs whole chunks open-loop (n_action_steps from training, 30 = 1.5 s).
# A float runs inference every step and averages the overlapping chunks with weights
# exp(-coeff * i), i = 0 for the OLDEST prediction - so a NEGATIVE coeff favours the newest,
# which keeps the final approach closed-loop on the image. At chunk 30, -0.05 gives the newest
# prediction ~5% weight and a mean prediction age of ~0.4 s.
#
# None, because ensembling STALLED the first bench test (2026-11-05): the tip stopped 2-3 mm
# short, at the pre-touch point. The demos hold still there for the vision-correction
# checkpoint (1309 mid-episode holds of up to 1 s), so from that pose the policy's FIRST
# predicted step is "hold" - and ensembling only ever executes first steps. A whole chunk
# carries the motion after the hold, so the arm moves through it.
TEMPORAL_ENSEMBLE_COEFF: float | None = None

# Per-step clamps (deg), just above the largest step in the 200 training episodes
# (coarse max 0.956, precision max 0.0167; p99 0.61 and 0.014): they never touch
# demonstrated motion and stop a runaway.
MAX_REL_TARGET = 1.0
PRECISION_MAX_REL_TARGET = 0.02
# Row "precision off": the flag no longer tightens the clamp or reaches the robot. It is still
# logged and still ends the episode, so runs with and without this stop in the same way.
IGNORE_PRECISION = False
# The commanded tip may go this far below the dish height map before the episode is stopped.
# The demos touched AT the surface (touch_offset_mm 0); the map drifts by up to ~40 um a day.
PRESS_LIMIT_MM = 0.1
# The episode ends once the policy has been in precision mode and the commanded tip has moved
# less than SETTLED_UM_PER_STEP per step for SETTLED_S (the demos end with a 0.5-1.5 s pause).
SETTLED_S = 1.0
SETTLED_UM_PER_STEP = 1.0
SUCCESS_UM = 50.0
DEVICE = "cuda"
# METHOD "policy": keep the camera frame the policy was given at each policy call (every
# n_action_steps steps in chunk mode; LIVE_EVERY_ENSEMBLE when ensembling) - after any
# IMAGE_SHIFT_PX, before the marker is drawn - in memory, and write them once the episode is over
# as epNNN/live/stepNNNN.png + live/index.json, so the 20 Hz loop does no disk I/O. For offline
# probes on the frames the policy really saw (raw vs AV1), and rollout figures. Added 2026-11-05,
# after the no-decoy ACT test, which has no such frames; it does not change what the policy sees.
SAVE_LIVE_FRAMES = True
LIVE_EVERY_ENSEMBLE = 10

# --- METHOD = "human" ---
OPERATOR = "experienced"  # names the results folder: "experienced", "novice1", "embryology", ...
HUMAN_TIME_S = 120.0  # as in record_microscope.py
# The SpaceMouse gains of teleoperate_microscope.py and record_microscope.py.
GAIN_TR = 50.0
GAIN_TR_FINE = 5.0
# -----------------------------------------------------------------


def deaf() -> dict:
    """Events for the scripted moves between episodes, NOT wired to the keyboard. The listener is
    global, so arrow keys pressed in any window cut transits short twice on 2026-11-05 (16.8 and
    25.2 deg from the start pose). Esc still stops the session between episodes."""
    return {"exit_early": False, "rerecord_episode": False, "stop_recording": False}


def session_name() -> str:
    name = {"policy": EVAL_NAME, "human": f"human_{OPERATOR}"}.get(METHOD, METHOD)
    if METHOD == "policy" and IGNORE_PRECISION:
        name += "_precision_off"
    return f"{name}_shift{IMAGE_SHIFT_PX}" if IMAGE_SHIFT_PX else name


class ObservationTap:
    """Sits on ``robot.get_observation``, so every reader gets the same view of an episode.

    The policy, the record_loop, the detector's ``grab`` and the human's display all read through
    it, so they cannot disagree about the shift. It also logs a timestamp and the joints on each
    read, which gives every METHOD the same trajectory record.
    """

    def __init__(self, robot, camera_key: str):
        self._read = robot.get_observation
        self._key = camera_key
        self.shift = (0, 0)
        self._t0 = 0.0
        self._t: list[float] | None = None
        self._q: list[list[float]] = []
        robot.get_observation = self.read

    def read(self) -> dict:
        obs = self._read()
        if self.shift != (0, 0):
            obs[self._key] = shift_frame(obs[self._key], *self.shift)
        if self._t is not None:
            self._t.append(time.perf_counter() - self._t0)
            self._q.append([float(obs[f"joint_{i + 1}.pos"]) for i in range(6)])
        return obs

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._t, self._q = [], []

    def stop(self) -> dict:
        out = {"obs_t": np.asarray(self._t or []), "obs_q": np.asarray(self._q)}
        self._t = None
        return out


def load_policy(model: str):
    """Policy + processors from a Hub repo or a local folder; refuses absolute-action checkpoints."""
    path = Path(model)
    if not path.exists():
        from huggingface_hub import snapshot_download

        path = Path(snapshot_download(model))
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg.device = DEVICE
    if cfg.type == "act" and TEMPORAL_ENSEMBLE_COEFF is not None:
        cfg.temporal_ensemble_coeff = TEMPORAL_ENSEMBLE_COEFF
        cfg.n_action_steps = 1
    train_cfg = json.loads((path / "train_config.json").read_text(encoding="utf-8"))
    dataset = train_cfg["dataset"]["repo_id"]
    if not dataset.endswith("_delta"):
        sys.exit(f"{model} was trained on {dataset}, not a delta dataset: this script sends deltas.")
    policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
    policy.eval()
    # Image-only policies trained on camera-axis tip motion (convert_to_tip_actions.py); IK turns
    # each action into a joint target in rollout_episode.
    policy.tip_actions = dataset.endswith("_tip_delta")
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=str(path), preprocessor_overrides={"device_processor": {"device": DEVICE}}
    )
    logging.info(f"Policy {cfg.type} from {path} (trained on {dataset}, {'tip' if policy.tip_actions else 'joint'} actions)")
    return policy, pre, post


def rollout_episode(
    robot, policy, pre, post, obs_proc, obs_features, planner, events, live_key: str | None = None
) -> dict:
    """Run the policy until it settles, the time runs out, a check fails or the operator stops it.

    With ``live_key``, the result's "live" holds (step, frame, joints) at each policy call: the
    camera frame before the marker overlay, copied in memory (see SAVE_LIVE_FRAMES)."""
    kin, checker, calib = planner.kin, planner.checker, planner.calib
    fps = planner.lim.fps
    policy.reset()
    pre.reset()
    post.reset()
    log = {k: [] for k in ("t", "q_cmd", "q_meas", "delta", "precision", "clamped", "backstop_cut")}
    n_steps = int(getattr(policy.config, "n_action_steps", 1))
    live_every = n_steps if n_steps > 1 else LIVE_EVERY_ENSEMBLE
    live = []
    reason, settled, ever_precise = "time limit", 0, False
    prev_tip = None
    # Integrate onto the last goal SENT, not the controller's live setpoint: the training deltas
    # are differences between successive commanded goals, and GetRtTargetJointPos() lags the
    # goal while a move is in flight. Integrating onto it dropped the unfinished part of every
    # step - the first bench test (2026-11-05) commanded only 54% of what the policy asked for
    # and never reached the dish. The arm is at rest after the transit, so the setpoint is the goal.
    q_goal = np.asarray(robot.robot.GetRtTargetJointPos(), dtype=float)
    tip_mode = getattr(policy, "tip_actions", False)
    axes = camera_axes(calib) if tip_mode else None
    t_goal = kin.tip(q_goal) if tip_mode else None
    t_start = time.perf_counter()
    while time.perf_counter() - t_start < EPISODE_TIME_S:
        t0 = time.perf_counter()
        # Esc only: the keyboard listener is global, and stray arrow keys from other windows cut
        # three episodes and transits short on 2026-11-05.
        if events["stop_recording"]:
            reason = "operator"
            break
        raw = robot.get_observation()
        step = len(log["t"])
        if live_key is not None and step % live_every == 0:
            # Copied before the overlay step can draw on it; the copy is never fed back.
            live.append((step, raw[live_key].copy(), [float(raw[f"joint_{j + 1}.pos"]) for j in range(6)]))
        obs = obs_proc(raw)
        frame = build_dataset_frame(obs_features, obs, prefix=OBS_STR)
        action = predict_action(frame, policy, torch.device(DEVICE), pre, post, False, TASK, robot.name)
        a = action.squeeze(0).float().cpu().numpy()
        precise = bool(a[6] > 0.5)
        ever_precise |= precise
        normal_clamp = PRECISION_MAX_REL_TARGET if precise else MAX_REL_TARGET
        clamp = MAX_REL_TARGET if IGNORE_PRECISION else normal_clamp
        if tip_mode:
            # Integrate onto the goal tip pose; the joint clamp below still bounds each step.
            t_next, q_ik = tip_action_to_joints(kin, t_goal, q_goal, a[:6], axes)
            if q_ik is None:
                reason = "stopped: no IK solution for the commanded tip pose"
                break
            joint_step = q_ik - q_goal
        else:
            joint_step = a[:6]
        delta = np.clip(joint_step, -clamp, clamp)
        q_cmd = q_goal + delta

        tip = kin.tip(q_cmd)[:3, 3]
        height = float(tip[2] - calib.surface_z_mm)
        if height < -PRESS_LIMIT_MM:
            reason = f"stopped: commanded tip {height * 1000:.0f} um below the dish surface"
            break
        clear = checker.clearance(q_cmd)
        if clear.min_mm < 0:
            reason = f"stopped: {clear.where} within margin of {clear.obstacle} ({clear.min_mm:.1f} mm)"
            break

        sent = robot.send_action(
            {
                **{f"joint_{i + 1}.pos": float(q_cmd[i]) for i in range(6)},
                "precision.state": float(precise and not IGNORE_PRECISION),
            }
        )
        # What was actually sent: the robot's backstop clamp (measured from the lagging setpoint)
        # may have cut the step, and the next delta must start from where the goal really is.
        q_goal = np.array([sent[f"joint_{i + 1}.pos"] for i in range(6)], dtype=float)
        if tip_mode:
            # A clamped step did not reach t_next: carry on from where the goal really is.
            t_goal = t_next if np.abs(q_goal - q_ik).max() < 1e-6 else kin.tip(q_goal)
        log["backstop_cut"].append(bool(np.abs(q_goal - q_cmd).max() > 1e-6))
        log["t"].append(t0 - t_start)
        log["q_cmd"].append(q_cmd)
        log["q_meas"].append(np.asarray(robot.robot.GetRtJointPos(), dtype=float))
        log["delta"].append(a[:6])
        log["precision"].append(precise)
        # Against the normal clamp in every run, so "precision off" shows what the flag would have done.
        log["clamped"].append(bool(np.any(np.abs(joint_step) > normal_clamp)))

        step_um = 0.0 if prev_tip is None else float(np.linalg.norm(tip - prev_tip)) * 1000
        prev_tip = tip
        settled = settled + 1 if (ever_precise and step_um < SETTLED_UM_PER_STEP) else 0
        if settled >= SETTLED_S * fps:
            reason = "settled"
            break
        precise_sleep(max(1 / fps - (time.perf_counter() - t0), 0.0))
    events["exit_early"] = False
    # The policy was trained on 20 Hz steps: a slower loop applies the same deltas less often.
    hz = 1 / float(np.median(np.diff(log["t"]))) if len(log["t"]) > 2 else float("nan")
    if hz < 0.9 * fps:
        logging.warning(f"Control loop ran at {hz:.1f} Hz, below the {fps} Hz the policy was trained at.")
    return {
        "reason": reason,
        "duration_s": round(time.perf_counter() - t_start, 2),
        "loop_hz": round(hz, 1),
        "log": log,
        "live": live,
    }


def harness_commit() -> str:
    """The commit this script ran from, "+dirty" if the working tree differs."""
    import subprocess

    def git(*a):
        return subprocess.run(["git", *a], capture_output=True, text=True, cwd=Path(__file__).parent).stdout.strip()

    sha = git("rev-parse", "--short", "HEAD") or "unknown"
    return sha + ("+dirty" if git("status", "--porcelain", "--untracked-files=no") else "")


def save_live(ep_dir: Path, live: list, target) -> None:
    """Write the frames rollout_episode kept, after the episode (no I/O in the control loop)."""
    if not live:
        return
    import cv2

    d = ep_dir / "live"
    d.mkdir(parents=True, exist_ok=True)
    index = []
    for step, img, q in live:
        name = f"step{step:04d}.png"
        cv2.imwrite(str(d / name), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        index.append({"step": step, "file": name, "joints_deg": q})
    (d / "index.json").write_text(
        json.dumps({"target_px": [float(v) for v in target], "frames": index}, indent=1),
        encoding="utf-8",
        newline="\n",
    )


def measure_final(robot, arm, planner, bench, background, target_px, shift=(0, 0)) -> tuple[dict, np.ndarray]:
    """Tip vs target by forward kinematics and by the tip detector, after the arm stops.

    ``robot`` supplies the frames and ``arm`` the controller connection (they differ for "human").
    """
    kin, calib = planner.kin, planner.calib
    arm.WaitIdle(timeout=10)
    q = np.asarray(arm.GetRtJointPos(), dtype=float)
    tip = kin.tip(q)[:3, 3]
    um_per_px = calib.mm_per_px * 1000
    fk_px = calib.xy_to_pixel(tip[:2])[0] + np.asarray(shift, dtype=float)
    final = grab(robot, bench.camera_key)
    det = detect_tip(final, background)
    out = {
        "fk_tip_px": np.round(fk_px, 2).tolist(),
        "fk_error_um": round(float(np.linalg.norm(fk_px - target_px)) * um_per_px, 1),
        "tip_height_um": round(float(tip[2] - calib.surface_z_mm) * 1000, 1),
        "final_joints_deg": np.round(q, 4).tolist(),
    }
    if det is not None:
        out["vision_tip_px"] = [round(det.px[0], 2), round(det.px[1], 2)]
        out["vision_error_um"] = round(float(np.linalg.norm(np.asarray(det.px) - target_px)) * um_per_px, 1)
        out["vision_score"] = round(det.score, 2)
    return out, final


def run_scripted(robot, teleop, plan, planner, events, processors) -> dict:
    """The plan as it stands: no image feedback, so it is only as good as the calibration."""
    run_plan(robot, teleop, plan.joints, plan.precision, planner.lim.fps, events, processors)
    reason = "operator" if events["exit_early"] or events["stop_recording"] else "finished"
    events["exit_early"] = False
    return {"reason": reason}


def run_expert(robot, teleop, plan, planner, key, background, events, processors, ep_dir) -> dict:
    """The collection controller. A recorder discards an aborted episode; a test counts it."""
    try:
        extra = run_corrected_episode(
            robot, teleop, plan, planner, key, background, events, processors, None, ep_dir
        )
    except EpisodeAbortedError as e:
        events["exit_early"] = events["rerecord_episode"] = False
        return {"reason": f"stopped: {e}"}
    events["exit_early"] = False
    return {"reason": "finished", "corrections": extra["corrections"]}


def human_start_joints(planner, plan) -> np.ndarray:
    """The episode's start position, at the planned touch orientation.

    A person cannot rotate in these trials, so they keep the orientation they start with. The
    start's own random tilt can put the final approach too close to the illuminator.
    """
    start = np.asarray(plan.start_joints, dtype=float)
    pose = planner.kin.tip(np.asarray(plan.joints[-1], dtype=float)).copy()
    pose[:3, 3] = planner.kin.tip(start)[:3, 3]
    q = planner.kin.ik(pose, start)
    if q is None or not planner.checker.is_clear(q):
        logging.warning("No clear start at the touch orientation: using the planned start.")
        return start
    return q


def follow_path(teleop, path, fps: int) -> None:
    """A collision-checked joint path over the SpaceMouse's control connection, paced as run_plan does."""
    for q in path:
        t0 = time.perf_counter()
        teleop.robot.MoveJoints(*(float(v) for v in q))
        precise_sleep(max(1 / fps - (time.perf_counter() - t0), 0.0))


def human_go_home(teleop, planner, home, fps: int) -> None:
    q_now = np.asarray(teleop.robot.GetRtJointPos(), dtype=float)
    if np.abs(q_now - home).max() < record_auto.START_TOLERANCE_DEG:
        return
    follow_path(teleop, planner.plan_transit(q_now, home, retreat=True), fps)
    wait_settled(teleop, home, SETTLE_TIMEOUT_S)


def show_frame(view, robot, obs_proc, key: str, title: str) -> None:
    view.show(obs_proc(robot.get_observation())[key], title=title)


def wait_for_operator(view, robot, obs_proc, key: str, title: str, events) -> None:
    """Keep the live view going until the right arrow (or Esc)."""
    events["exit_early"] = False
    while not (events["exit_early"] or events["stop_recording"] or view.closed):
        show_frame(view, robot, obs_proc, key, title)
    events["exit_early"] = False


def run_human(teleop, guard, view, robot, obs_proc, key, trial: str, events) -> dict:
    """One trial: the person drives until they press the right arrow, the guard trips or time runs out."""
    guard.reset()
    teleop.resume()
    t_start = time.perf_counter()
    reason = "time limit"
    while (elapsed := time.perf_counter() - t_start) < HUMAN_TIME_S:
        if guard.tripped:
            reason = guard.tripped
            break
        if events["exit_early"] or events["stop_recording"] or view.closed:
            reason = "operator"
            break
        mode = "FINE" if teleop.precision_active else "COARSE"
        floor = " | ON THE DISH" if guard.floor_hit else ""
        show_frame(
            view,
            robot,
            obs_proc,
            key,
            f"{trial} | {elapsed:.0f} s | {mode} (Enter){floor} | right arrow when the tip is on the dish in the ring",
        )
    teleop.pause()
    events["exit_early"] = False
    return {"reason": reason, "floor_hit": guard.floor_hit}


def operator_backs_out(teleop, guard, view, robot, obs_proc, key, events) -> None:
    """After a trip the arm can be too close to plan away, so the person clears it (the floor still holds)."""
    guard.reset()
    guard.check_clearance = False
    teleop.resume()
    wait_for_operator(
        view, robot, obs_proc, key, "Back the pipette out, away from what it hit, then right arrow", events
    )
    teleop.pause()
    guard.check_clearance = True


def main() -> None:
    if METHOD not in ("policy", "scripted", "expert", "human"):
        sys.exit(f"METHOD must be policy, scripted, expert or human, not {METHOD!r}")
    human = METHOD == "human"
    record_auto.DISPLAY_DATA = False  # no Rerun stream from run_corrected_episode
    init_logging()
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    try:
        pos = check_focus(bench.section("scope"))
    except FocusMismatchError as e:
        sys.exit(f"\nREFUSING TO START: {e}\n")
    print(f"Focus check passed: K-Cube at {pos:.4f} mm, as calibrated.")
    policy, pre, post = load_policy(MODEL) if METHOD == "policy" else (None, None, None)

    planner = build_planner(bench)
    stage, model, hmap = connect_stage(bench, planner.calib)
    stager = StagePlanner(planner, model, hmap, planner.calib)
    home = np.asarray(require_home_joints(RIG), dtype=float)
    fps = planner.lim.fps

    # The planned paths keep to the clamps they were recorded under; the policy's are tighter.
    scripted = METHOD in ("scripted", "expert")
    robot = make_robot_from_config(
        rig_config(
            RIG,
            monitor_mode=human,  # the SpaceMouse teleop owns the control connection
            relative_actions=False,
            # The policy's clamp is applied in rollout_episode; here it is a 2x runaway backstop,
            # because the robot measures it from the live setpoint, which lags the goal.
            max_relative_target=record_auto.MAX_REL_TARGET if scripted else 2 * MAX_REL_TARGET,
            precision_max_relative_target=record_auto.PRECISION_MAX_REL_TARGET
            if scripted
            else 2 * PRECISION_MAX_REL_TARGET,
        )
    )
    guard, view = None, None
    if human:
        teleop = Meca500Spacemouse(
            Meca500SpacemouseConfig(
                id="meca500_spacemouse",
                gain_tr=GAIN_TR,
                gain_tr_fine=GAIN_TR_FINE,
                axis_signs=axis_signs(RIG),
                home_joints=[float(v) for v in home],
                translation_only=True,
                start_paused=True,
                home_hotkey=False,
            )
        )
        guard = OperatorGuard(planner)
        teleop.velocity_filter = guard.filter
    else:
        teleop = Meca500Scripted(Meca500ScriptedConfig(id="meca500_scripted"))
    tap = ObservationTap(robot, bench.camera_key)
    target_cfg = bench.section("target")
    overlay = TargetOverlayStep(camera_key=bench.camera_key, style=bench.target_style())
    teleop_proc, robot_proc, _ = make_default_processors()
    obs_proc = RobotProcessorPipeline(
        steps=[overlay], to_transition=observation_to_transition, to_output=transition_to_observation
    )
    processors = (teleop_proc, robot_proc, obs_proc)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR, use_video=True)

    out_dir = Path("outputs/eval_auto") / f"{session_name()}_{time.strftime('%Y%m%d-%H%M%S')}"
    out_dir.mkdir(parents=True)
    commit = harness_commit()
    (out_dir / "config.json").write_text(
        json.dumps(
            {
                k: v
                for k, v in globals().items()
                if k.isupper() and isinstance(v, (str, int, float, type(None)))
            }
            | {"harness_commit": commit},
            indent=2,
        ),
        encoding="utf-8",
        newline="\n",
    )
    results = []

    robot.connect()
    if not human:
        robot.configure()
    teleop.connect()
    # Whichever connection is in control. Both have .robot, so wait_settled takes either.
    controller = teleop if human else robot
    arm = controller.robot
    listener, events = init_keyboard_listener()
    try:
        make_ready(arm)
        if human:
            arm.SetBlending(100)
            if robot.config.default_joint_vel:
                arm.SetJointVel(robot.config.default_joint_vel)
            arm.SetRealTimeMonitoring("all")
            # Imported here: matplotlib is only needed for this mode, and a window for no other.
            from live_view import LiveView

            view = LiveView("eval_auto")
        if not stage.is_homed:
            if np.abs(np.array(arm.GetRtJointPos()) - home).max() > 0.2:
                sys.exit(
                    "The stage is not homed. Home it in Kinesis with the arm at home (or rerun with the arm at home)."
                )
            stage.home()
        stage_xy = stage.position()
        stager.place(stage_xy)
        if human:
            human_go_home(teleop, planner, home, fps)
            q_now = np.asarray(arm.GetRtJointPos(), dtype=float)
        else:
            q_now = go_home(robot, teleop, planner, home, deaf(), processors)
        retreat = False
        for i in range(FIRST_EPISODE, NUM_EPISODES):
            if events["stop_recording"]:
                break
            try:
                new_xy, plan = stager.plan_episode(SEED_BASE + i)
                start_q = human_start_joints(planner, plan) if human else plan.start_joints
                stager.place(stage_xy)
                stager.check_parked_arm(start_q, stage_xy, new_xy)
                try:
                    transit = planner.plan_transit(q_now, start_q, retreat=retreat)
                except PlanningError:
                    if not human:
                        raise
                    operator_backs_out(teleop, guard, view, robot, obs_proc, bench.camera_key, events)
                    q_now = np.asarray(arm.GetRtJointPos(), dtype=float)
                    transit = planner.plan_transit(q_now, start_q, retreat=retreat)
            except PlanningError as e:
                logging.error(str(e))
                break
            overlay.set_target(None)
            log_say(f"Moving to start of episode {i}", False)
            if human:
                follow_path(teleop, transit, fps)
            else:
                run_plan(robot, teleop, transit, np.zeros(len(transit)), fps, deaf(), processors)
            wait_settled(controller, start_q, SETTLE_TIMEOUT_S)
            stage_xy = stage.move_to(new_xy)
            surface_z = stager.place(stage_xy)
            target = np.asarray(plan.target_px, dtype=float)
            shift = sample_shift(
                SEED_BASE + i,
                IMAGE_SHIFT_PX,
                target,
                target_cfg["field_centre_px"],
                target_cfg["field_radius_px"] * target_cfg["field_radius_frac"],
            )
            tap.shift = shift
            background, _ = settled_background(robot, bench.camera_key)
            ep_dir = out_dir / f"ep{i:03d}"
            save_raw(ep_dir, "background", background)

            overlay.set_target(plan.target_px)
            trial = f"Trial {i + 1} of {NUM_EPISODES}"
            if human:
                wait_for_operator(
                    view,
                    robot,
                    obs_proc,
                    bench.camera_key,
                    f"{trial}: press the right arrow to start",
                    events,
                )
                if events["stop_recording"] or view.closed:
                    break
            log_say(f"{METHOD} episode {i}", False)
            t_drive = time.perf_counter()
            tap.start()
            if METHOD == "policy":
                run = rollout_episode(
                    robot, policy, pre, post, obs_proc, obs_features, planner, events,
                    live_key=bench.camera_key if SAVE_LIVE_FRAMES else None,
                )
            elif METHOD == "scripted":
                run = run_scripted(robot, teleop, plan, planner, events, processors)
            elif METHOD == "expert":
                run = run_expert(
                    robot, teleop, plan, planner, bench.camera_key, background, events, processors, ep_dir
                )
            else:
                run = run_human(teleop, guard, view, robot, obs_proc, bench.camera_key, trial, events)
            traj = tap.stop()
            duration_s = round(time.perf_counter() - t_drive, 2)
            m, final = measure_final(robot, arm, planner, bench, background, target, shift)
            overlay.set_target(None)
            tap.shift = (0, 0)
            save_raw(ep_dir, "final", final)
            save_raw(ep_dir, "final_overlay", draw_target(final, target, bench.target_style()))
            log = run.pop("log", {})
            np.savez_compressed(ep_dir / "traj.npz", **traj, **{k: np.asarray(v) for k, v in log.items()})
            live = run.pop("live", [])
            save_live(ep_dir, live, target)

            row = {
                "episode": i,
                "seed": SEED_BASE + i,
                "method": METHOD,
                "operator": OPERATOR if human else None,
                "image_shift_px": list(shift),
                "target_px": [round(float(v), 2) for v in target],
                "stage_xy_mm": [round(float(v), 4) for v in stage_xy],
                "surface_z_mm": round(surface_z, 4),
                "reason": run["reason"],
                "duration_s": duration_s,
                "steps": len(traj["obs_t"]),
                "clamped_steps": int(np.sum(log["clamped"])) if log else None,
                "backstop_cut_steps": int(np.sum(log["backstop_cut"])) if log and "backstop_cut" in log else None,
                "live_frames": len(live),
                "harness_commit": commit,
                **{k: v for k, v in run.items() if k not in ("reason", "duration_s")},
                **m,
            }
            results.append(row)
            with (out_dir / "results.jsonl").open("a", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(row) + "\n")
            logging.info(
                f"episode {i}: {run['reason']} after {duration_s} s | FK error {m['fk_error_um']:.0f} um, "
                f"vision {m.get('vision_error_um', float('nan')):.0f} um, tip {m['tip_height_um']:+.0f} um"
            )
            if arm.GetStatusRobot().error_status:
                raise RuntimeError("Robot reported an error during the episode; stopping.")
            q_now = np.array(arm.GetRtJointPos())
            retreat = True

        events["exit_early"] = False
        try:
            if human:
                human_go_home(teleop, planner, home, fps)
            else:
                go_home(robot, teleop, planner, home, deaf(), processors)
        except (RuntimeError, PlanningError) as e:
            logging.error(f"Could not return home ({e}); the arm holds its current pose.")
    finally:
        overlay.set_target(None)
        if view is not None:
            with contextlib.suppress(Exception):
                view.close()
        with contextlib.suppress(Exception):
            robot.disconnect()
        with contextlib.suppress(Exception):
            teleop.disconnect()
        with contextlib.suppress(Exception):
            stage.disconnect()
        if not is_headless() and listener is not None:
            with contextlib.suppress(Exception):
                listener.stop()

    if results:
        fk = np.array([r["fk_error_um"] for r in results])
        vis = np.array([r.get("vision_error_um", np.nan) for r in results])
        print(
            f"\n{len(results)} episodes | FK error median {np.median(fk):.0f} um, "
            f"<= {SUCCESS_UM:.0f} um in {int(np.sum(fk <= SUCCESS_UM))} | vision median {np.nanmedian(vis):.0f} um, "
            f"<= {SUCCESS_UM:.0f} um in {int(np.sum(vis <= SUCCESS_UM))} | judge success by eye on epNNN/final_overlay.png\n"
            f"-> {out_dir}"
        )


if __name__ == "__main__":
    main()
