"""Tests for the automated-collection stack: kinematics, collision, calibration, overlay,
scripted teleop and planner. Everything here runs without hardware or the bench CAD."""

import numpy as np
import pytest

from lerobot.robots.meca500.autocollect.bench import PlannerLimits
from lerobot.robots.meca500.autocollect.calibration import ScopeCalibration
from lerobot.robots.meca500.autocollect.collision import (
    BoxField,
    CollisionChecker,
    CylinderField,
    RobotPoints,
    TransformedField,
    fit_rigid,
)
from lerobot.robots.meca500.autocollect.kinematics import (
    Meca500Kinematics,
    make_transform,
    matrix_to_pose,
    pose_to_matrix,
)
from lerobot.robots.meca500.autocollect.overlay import TargetOverlayStep, TargetStyle, draw_target
from lerobot.robots.meca500.autocollect.planner import Planner, PlanningError
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500.meca500 import Meca500
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted

TOOL = make_transform(None, [0, 0, 60])  # a 60 mm straight pipette along the flange axis


@pytest.fixture(scope="module")
def kin():
    return Meca500Kinematics(tool=TOOL)


# ----------------------------------------------------------------------------- kinematics


def test_fk_zero_matches_the_meca_documented_zero_pose():
    k = Meca500Kinematics()
    np.testing.assert_allclose(matrix_to_pose(k.flange([0] * 6)), [190, 0, 308, 0, 90, 0], atol=1e-9)


def test_pose_matrix_round_trip():
    pose = [12.0, -40.0, 100.0, 20.0, -35.0, 70.0]
    np.testing.assert_allclose(matrix_to_pose(pose_to_matrix(pose)), pose, atol=1e-9)


@pytest.mark.parametrize("q", [[10, 20, -30, 40, 50, 60], [-90, 40, 20, 90, 80, -110], [0, 30, 25, 0, 20, 0]])
def test_ik_recovers_the_pose_from_a_nearby_seed(kin, q):
    q = np.array(q, float)
    target = kin.tip(q)
    sol = kin.ik(target, q + np.array([4, -4, 4, -4, 4, -4]))
    assert sol is not None
    np.testing.assert_allclose(kin.tip(sol), target, atol=1e-3)


def test_jacobian_matches_finite_differences(kin):
    q = np.array([5.0, 25.0, 10.0, 30.0, 40.0, -20.0])
    jac = kin.jacobian(q)
    eps = 1e-6
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = np.degrees(eps)
        dp = (kin.tip(q + dq)[:3, 3] - kin.tip(q)[:3, 3]) / eps
        np.testing.assert_allclose(jac[:3, i], dp, rtol=1e-4, atol=1e-4)


# ----------------------------------------------------------------------------- collision


def test_primitive_fields_signed_distance():
    box = BoxField(center=[0, 0, 0], size=[20, 20, 20])
    np.testing.assert_allclose(box.distance(np.array([[20, 0, 0], [0, 0, 0]])), [10, -10])
    cyl = CylinderField(base=[0, 0, 0], radius=5, height=10)
    np.testing.assert_allclose(cyl.distance(np.array([[10, 0, 5], [0, 0, 15]])), [5, 5])


def test_grid_field_agrees_with_analytic_box(tmp_path):
    trimesh = pytest.importorskip("trimesh")
    pytest.importorskip("scipy")
    from lerobot.robots.meca500.autocollect.collision import GridField

    path = tmp_path / "box.stl"
    trimesh.creation.box(extents=[40, 40, 40]).export(path)
    grid = GridField.from_meshes([path], pitch=1.0, pad=20)
    box = BoxField([0, 0, 0], [40, 40, 40])
    pts = np.array([[30, 0, 0], [0, 35, 5], [0, 0, 0], [100, 0, 0]])
    d_grid, d_box = grid.distance(pts), box.distance(pts)
    # conservative: never reports more clearance than there is, and close where it matters
    assert np.all(d_grid <= d_box + 1e-6)
    np.testing.assert_allclose(d_grid[:2], d_box[:2], atol=2.0)
    assert (tmp_path / next(p.name for p in tmp_path.iterdir() if p.suffix == ".npz")).exists()


def test_fit_rigid_recovers_a_known_transform_and_moves_fields_with_it():
    rng = np.random.default_rng(1)
    t = make_transform(pose_to_matrix([0, 0, 0, 1.0, -2.0, 3.0])[:3, :3], [5.0, -3.0, 2.0])
    cad = rng.uniform(-200, 200, size=(6, 3))
    measured = cad @ t[:3, :3].T + t[:3, 3]
    np.testing.assert_allclose(fit_rigid(cad, measured), t, atol=1e-9)

    moved = TransformedField(BoxField([0, 0, 0], [10, 10, 10]), make_transform(None, [100, 0, 0]))
    np.testing.assert_allclose(moved.distance(np.array([[100.0, 0, 0], [0.0, 0, 0]])), [-5, 95])


def _checker(kin, fields, **kw):
    return CollisionChecker(kin=kin, fields=fields, points=RobotPoints.build(kin), **kw)


def test_checker_flags_a_box_at_the_flange_and_clears_a_distant_one(kin):
    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    flange = kin.flange(q)[:3, 3]
    assert not _checker(kin, [BoxField(flange, [20, 20, 20])]).is_clear(q)
    assert _checker(kin, [BoxField([-400, 0, 0], [20, 20, 20])]).is_clear(q)


def test_check_path_catches_a_collision_between_waypoints(kin):
    # J1 swings the arm from +y to -y; a box on the +x axis is only hit mid-swing.
    a, b = np.array([90.0, 0, 0, 0, 0, 0]), np.array([-90.0, 0, 0, 0, 0, 0])
    box = BoxField([200, 0, 300], [30, 30, 30])
    checker = _checker(kin, [box])
    assert checker.is_clear(a) and checker.is_clear(b)
    assert checker.check_path(np.vstack([a, b])).min_mm < 0


def test_tip_exemption_lets_the_tip_touch_but_not_the_holder(kin, tmp_path):
    trimesh = pytest.importorskip("trimesh")
    tool_path = tmp_path / "tool.stl"
    rod = trimesh.creation.cylinder(radius=1.0, height=60)
    rod.apply_translation([0, 0, 30])  # flange frame, rod from z=0 to z=60
    rod.export(tool_path)
    q = np.array([0.0, 30, 25, 0, 20, 0])
    tip = kin.tip(q)[:3, 3]
    floor = BoxField(tip - [0, 0, 5], [400, 400, 10])  # top face exactly at the tip
    pts = RobotPoints.build(kin, tool_path)
    assert CollisionChecker(kin, [floor], pts, link_margin_mm=0, tool_margin_mm=2, tip_exempt_mm=10).is_clear(
        q
    )
    assert not CollisionChecker(
        kin, [floor], pts, link_margin_mm=0, tool_margin_mm=2, tip_exempt_mm=0
    ).is_clear(q)


# ----------------------------------------------------------------------------- calibration


def test_affine_calibration_recovers_a_mirrored_rotated_map():
    rng = np.random.default_rng(0)
    th = np.radians(270)
    lin = 0.003 * np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]) @ np.diag([1, -1])
    offset = np.array([150.0, -20.0])
    px = rng.uniform([0, 0], [427, 640], size=(12, 2))
    xy = px @ lin.T + offset
    cal = ScopeCalibration.fit(px, xy, surface_z_mm=40.0, image_size=(427, 640))
    np.testing.assert_allclose(cal.pixel_to_xy(px), xy, atol=1e-9)
    np.testing.assert_allclose(cal.xy_to_pixel(xy), px, atol=1e-6)
    assert cal.residual_px_rms < 1e-6
    assert cal.mm_per_px == pytest.approx(0.003)


def test_calibration_save_load(tmp_path):
    px = np.array([[0, 0], [100, 0], [0, 100], [100, 100.0]])
    cal = ScopeCalibration.fit(px, px * 0.01, 5.0, (427, 640))
    cal.save(tmp_path / "c.json")
    assert ScopeCalibration.load(tmp_path / "c.json") == cal


# ----------------------------------------------------------------------------- overlay


def test_overlay_draws_at_the_requested_pixel_and_leaves_the_rest():
    img = np.full((640, 427, 3), 128, np.uint8)
    out = draw_target(img, (200, 300), TargetStyle(color=(0, 255, 0)))
    assert tuple(out[300, 200]) == (0, 255, 0)  # cross centre
    assert tuple(out[300, 214]) == (0, 255, 0)  # on the ring (radius 14)
    assert tuple(out[10, 10]) == (128, 128, 128)
    assert tuple(img[300, 200]) == (128, 128, 128)  # input not mutated


def test_overlay_step_only_draws_when_a_target_is_set():
    step = TargetOverlayStep(camera_key="microscope_cam")
    obs = {"microscope_cam": np.zeros((640, 427, 3), np.uint8), "joint_1.pos": 0.0}
    assert step.observation(dict(obs))["microscope_cam"].sum() == 0
    step.set_target((100, 100))
    assert step.observation(dict(obs))["microscope_cam"].sum() > 0


def test_overlay_step_resamples_on_reset_when_given_a_sampler():
    targets = iter([(10.0, 20.0), (30.0, 40.0)])
    step = TargetOverlayStep(sampler=lambda: next(targets))
    assert step.target_px is None
    step.reset()
    assert step.target_px == (10.0, 20.0)
    step.reset()
    assert step.target_px == (30.0, 40.0)
    TargetOverlayStep(target_px=(1.0, 2.0)).reset()  # no sampler: reset leaves it alone


# ----------------------------------------------------------------------------- scripted teleop


def test_scripted_teleop_matches_the_robot_action_schema():
    teleop = Meca500Scripted(Meca500ScriptedConfig())
    robot = Meca500(Meca500Config(cameras={}))
    assert set(teleop.action_features) == set(robot.action_features)


def test_scripted_teleop_replays_then_holds_the_last_row():
    teleop = Meca500Scripted(Meca500ScriptedConfig())
    with pytest.raises(RuntimeError):
        teleop.get_action()
    joints = np.arange(18, dtype=float).reshape(3, 6)
    teleop.load_plan(joints, [0, 0, 1])
    actions = [teleop.get_action() for _ in range(5)]
    assert [a["joint_1.pos"] for a in actions] == [0, 6, 12, 12, 12]
    assert [a["precision.state"] for a in actions] == [0, 0, 1, 1, 1]
    assert teleop.done


# ----------------------------------------------------------------------------- planner

START_Q = [0.0, 10.0, 0.0, 0.0, 60.0, 0.0]
TOUCH_Q = [0.0, 30.0, 25.0, 0.0, 20.0, 0.0]


def _planner(kin, fields):
    touch = kin.tip(TOUCH_Q)[:3, 3]
    # a 1.3 x 1.9 mm field of view centred on the nominal touch point, mirrored like the Basler
    lin = 0.003 * np.array([[0, 1], [1, 0]])
    px = np.array([[0, 0], [426, 0], [0, 639], [426, 639.0]])
    xy = px @ lin.T + (touch[:2] - lin @ [213, 320])
    cal = ScopeCalibration.fit(px, xy, surface_z_mm=touch[2], image_size=(427, 640))
    limits = PlannerLimits(start_nominal_joints=START_Q, touch_nominal_joints=TOUCH_Q, max_attempts=50)
    checker = _checker(kin, fields, link_margin_mm=10, tool_margin_mm=3)
    return Planner(kin, checker, cal, limits), cal


def test_planner_produces_a_safe_plan_that_ends_on_the_target(kin):
    lamp = BoxField([-150, 0, 300], [60, 60, 60], name="lamp")
    planner, cal = _planner(kin, [lamp])
    plan = planner.sample_episode(seed=3)

    final_tip = kin.tip(plan.joints[-1])[:3, 3]
    np.testing.assert_allclose(final_tip, plan.target_xyz, atol=0.01)
    np.testing.assert_allclose(cal.xy_to_pixel(final_tip[:2])[0], plan.target_px, atol=0.5)
    assert plan.clearance.min_mm >= 0
    # the plan ends in precision mode and starts outside it
    assert plan.precision[0] == 0 and plan.precision[-1] == 1
    steps = np.abs(np.diff(plan.joints, axis=0)).max(axis=1)
    assert steps.max() <= planner.lim.max_joint_step_deg + 1e-9
    assert steps[plan.precision[1:] == 1].max() <= planner.lim.max_joint_step_precision_deg + 1e-9
    assert set(plan.meta()) >= {"seed", "target_px", "min_clearance_mm"}


def test_planner_is_reproducible_per_seed_and_varies_across_seeds(kin):
    planner, _ = _planner(kin, [BoxField([-150, 0, 300], [60, 60, 60])])
    a, b, c = planner.sample_episode(7), planner.sample_episode(7), planner.sample_episode(8)
    np.testing.assert_array_equal(a.joints, b.joints)
    assert a.target_px != c.target_px


def test_planner_keeps_the_target_while_resampling_start_and_path(kin):
    """Redrawing the target on every rejection would under-sample hard-to-reach parts of the
    image. The target must stay fixed across retries and only change after target_attempts."""
    from lerobot.robots.meca500.autocollect import planner as planner_mod

    planner, _ = _planner(kin, [BoxField([-150, 0, 300], [60, 60, 60])])
    planner.lim.target_attempts = 3
    seen = []
    real_try = planner._try_episode

    def flaky(rng, target_px=None):
        seen.append(tuple(target_px))
        if len(seen) < 5:  # reject the first four attempts
            raise planner_mod._RejectedSampleError("forced")
        return real_try(rng, target_px)

    planner._try_episode = flaky
    plan = planner.sample_episode(0)
    assert seen[0] == seen[1] == seen[2]  # same target for the first target_attempts tries
    assert seen[3] != seen[2] and seen[3] == seen[4]  # then replaced once, and kept again
    assert tuple(plan.target_px) == pytest.approx(seen[4])
    assert planner.rejections["target replaced (unreachable)"] == 1


def test_planner_refuses_when_every_path_is_blocked(kin):
    # a slab covering the whole work area just above the dish blocks every approach
    touch = kin.tip(TOUCH_Q)[:3, 3]
    slab = BoxField(touch + [0, 0, 30], [600, 600, 20], name="slab")
    planner, _ = _planner(kin, [slab])
    planner.lim.max_attempts = 5
    with pytest.raises(PlanningError):
        planner.sample_episode(0)


def test_record_auto_records_the_commanded_plan_with_the_marker(tmp_path, monkeypatch):
    """End-to-end through record_loop: scripted teleop -> Meca500 (mocked controller) ->
    overlay processor -> LeRobotDataset. The recorded action must be the plan itself and
    the recorded frame must carry the marker."""
    import sys
    from pathlib import Path
    from unittest.mock import MagicMock

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets.pipeline_features import (
        aggregate_pipeline_dataset_features,
        create_initial_features,
    )
    from lerobot.processor import (
        RobotProcessorPipeline,
        make_default_processors,
        observation_to_transition,
        transition_to_observation,
    )
    from lerobot.utils.feature_utils import combine_feature_dicts

    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts" / "meca500"))
    import record_auto

    class Cam:
        is_connected, height, width = True, 64, 48

        def read_latest(self, max_age_ms=0):
            return np.full((64, 48, 3), 100, np.uint8)

    robot = Meca500(Meca500Config(monitor_mode=False, cameras={}))
    robot.cameras = {"microscope_cam": Cam()}
    state = {"q": [0.0] * 6}
    robot.robot = MagicMock()
    robot.robot.GetJoints = MagicMock(side_effect=lambda: list(state["q"]))
    robot.robot.GetRtTargetJointPos = MagicMock(side_effect=lambda: list(state["q"]))
    robot.robot.MoveJoints = MagicMock(side_effect=lambda *q: state.update(q=list(q)))
    robot._connected = True

    overlay = TargetOverlayStep(camera_key="microscope_cam", style=TargetStyle(color=(0, 255, 0)))
    teleop_proc, robot_proc, _ = make_default_processors()
    obs_proc = RobotProcessorPipeline(
        steps=[overlay], to_transition=observation_to_transition, to_output=transition_to_observation
    )
    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_proc,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=False,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=obs_proc,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=False,
        ),
    )
    dataset = LeRobotDataset.create(
        "test/auto", 20, features, root=tmp_path / "ds", robot_type="meca500", use_videos=False
    )

    teleop = Meca500Scripted(Meca500ScriptedConfig())
    teleop.connect()
    plan = np.linspace([0, 0, 0, 0, 30, 0], [1, 2, 3, 4, 31, 6], 8)
    precision = np.array([0, 0, 0, 0, 0, 1, 1, 1.0])
    overlay.set_target((24, 32))
    events = {"exit_early": False, "rerecord_episode": False, "stop_recording": False}
    monkeypatch.setattr(record_auto, "TASK", "test")
    record_auto.run_plan(
        robot, teleop, plan, precision, 20, events, (teleop_proc, robot_proc, obs_proc), dataset
    )
    dataset.save_episode()
    dataset.finalize()

    actions = np.stack([dataset[i]["action"].numpy() for i in range(dataset.num_frames)])
    assert dataset.num_frames == len(plan)
    np.testing.assert_allclose(actions[:, :6], plan, atol=1e-5)
    np.testing.assert_allclose(actions[:, 6], precision)
    frame = dataset[0]["observation.images.microscope_cam"]  # (3, H, W) float in [0, 1]
    assert frame[1, 32, 24] > 0.95 and frame[0, 32, 24] < 0.05  # green marker at (u=24, v=32)


def test_transit_retreats_and_reaches_the_next_start(kin):
    planner, _ = _planner(kin, [BoxField([-150, 0, 300], [60, 60, 60])])
    plan = planner.sample_episode(1)
    qs = planner.plan_transit(plan.joints[-1], START_Q)
    np.testing.assert_allclose(qs[-1], START_Q, atol=1e-9)
    # first move after the touch is back along the pipette axis, i.e. away from the dish
    assert kin.tip(qs[5])[2, 3] > kin.tip(qs[0])[2, 3]


# ----------------------------------------------------------------------------- tip detection


def _microscope_frames(tip=(220.0, 300.0), angle_deg=70.0, seed=0):
    """A synthetic Basler-like pair: bright disc with clutter, and the same plus a dark pipette."""
    import cv2

    rng = np.random.default_rng(seed)
    bg = np.zeros((640, 427, 3), np.uint8)
    cv2.circle(bg, (213, 320), 205, (240, 200, 60), -1)
    for _ in range(300):  # scratches and specks
        x, y = rng.integers(20, 410), rng.integers(120, 520)
        cv2.circle(bg, (int(x), int(y)), int(rng.integers(1, 3)), (90, 70, 20), -1)
    frame = bg.copy()
    a = np.radians(angle_deg)
    shank = (int(tip[0] + 400 * np.cos(a)), int(tip[1] + 400 * np.sin(a)))  # runs off the field edge
    cv2.line(frame, (int(tip[0]), int(tip[1])), shank, (30, 25, 10), 7)
    return frame, bg


@pytest.mark.parametrize(
    "tip,angle", [((220.0, 300.0), 70.0), ((120.0, 250.0), 110.0), ((300.0, 420.0), 40.0)]
)
def test_tip_detection_finds_the_free_end_of_the_pipette(tip, angle):
    from lerobot.robots.meca500.autocollect.tip_detect import detect_tip

    frame, bg = _microscope_frames(tip, angle)
    det = detect_tip(frame, bg)
    assert det is not None
    assert np.hypot(det.px[0] - tip[0], det.px[1] - tip[1]) < 4.0  # within a few px of the drawn tip
    a = np.radians(angle)
    assert np.dot(det.axis, [-np.cos(a), -np.sin(a)]) > 0.95  # axis points from the shank to the tip


def test_tip_detection_reports_nothing_without_a_pipette():
    from lerobot.robots.meca500.autocollect.tip_detect import detect_tip

    _, bg = _microscope_frames()
    assert detect_tip(bg, bg) is None


# ----------------------------------------------------------------------------- contact sensing


def test_contact_rule_uses_torque_and_backstops():
    from lerobot.robots.meca500.autocollect.contact import ContactThresholds, evaluate

    th = ContactThresholds(torque_mnm=0.3, force_limit_n=0.05, torque_limit_mnm=3.0)
    base = np.zeros(6)
    quiet = np.array([0.01, -0.01, 0.005, 1e-4, -1e-4, 0.0])  # noise-level drift
    assert not evaluate(quiet, base, th).contact
    touch = np.array([0.0, 0.0, 0.0, 0.0, 4e-4, 0.0])  # 0.4 mNm step: a light touch
    r = evaluate(touch, base, th)
    assert r.contact and r.reason == "torque"
    shove = np.array([0.0, 0.0, -0.08, 0.0, 0.0, 0.0])  # force over the 0.05 N backstop
    assert evaluate(shove, base, th).reason == "force limit"


def test_wrench_stream_dedupes_and_detector_sees_a_step():
    import time as _time
    from types import SimpleNamespace

    from lerobot.robots.meca500.autocollect.contact import ContactDetector, WrenchStream

    state = {"t": [0.0, 0.0, 0.0], "n": 0}

    class FakeSensor:
        def read_frame(self):
            state["n"] += 1
            return SimpleNamespace(force=[0.0, 0.0, 0.0], torque=list(state["t"]))  # repeats until changed

    stream = WrenchStream(FakeSensor())
    try:
        _time.sleep(0.2)
        assert len(stream.window(1.0)) <= 2  # identical frames are dropped
        det = ContactDetector(stream)
        state["t"] = [0.0, 1e-5, 0.0]
        _time.sleep(0.05)
        det.capture_baseline(0.5)
        assert not det.check().contact
        state["t"] = [0.0, 1e-3, 0.0]  # 1 mNm step
        _time.sleep(0.25)
        assert det.check().contact
    finally:
        stream.stop()
