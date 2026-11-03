"""Record microscope pipetting demos with no human: a scripted expert drives the arm to a
digital target drawn on the microscope feed.

Per episode:
    1. plan  - sample a start pose, a target pixel and a randomised path; IK + collision-check
               every frame (autocollect/planner.py). Nothing moves until a plan passes.
    2. transit (NOT recorded) - back the tip out along the pipette, then a collision-checked
               joint move to the new start.
    3. record - the marker appears at the target pixel; the scripted teleop replays the plan
               as absolute MoveJoints targets, one per frame, ending with the tip touching the
               dish on the marker in precision mode. With VISION_CORRECTION the final approach
               stops twice (at the pre-touch point and CORRECTION_NEAR_MM before the touch),
               detects the tip and shifts the rest of the approach onto the target
               (autocollect/correction.py); the pauses are recorded, the detection is not.

The dataset schema is identical to the SpaceMouse recordings (joint_1..6.pos +
precision.state, microscope_cam), but the action is the COMMANDED target rather than a
position readback. Train on the delta version as usual:
    python scripts/meca500/convert_to_delta_actions.py   (point it at this dataset)

Each episode's seed, target pixel, start pose, clearance, the corrections made and the final
detected tip-to-target error go to autocollect_episodes.jsonl
in the dataset folder (NOT dataset features, so nothing leaks into the policy's inputs),
next to a copy of the bench config and scope calibration used.

Bring-up order (STAGE below): "hover" first - stops 5 mm above the dish at half speed -
and watch a few episodes with a hand on the e-stop; then "touch" on a dummy dish; then
unattended batches. Keys as in record: right arrow ends an episode early (it is then
discarded), left arrow re-records, Esc stops.

Prerequisites: check_kinematics.py passes, the bench has its STLs + tool + calibration,
check_demos_clearance.py finds no demo collisions, plan_preview.py looks right.
"""

import contextlib
import json
import logging
import shutil
import sys
import time
from dataclasses import replace

import numpy as np
from rigs import require_home_joints, rig_config

from lerobot.common.control_utils import sanity_check_dataset_name, sanity_check_dataset_robot_compatibility
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
from lerobot.datasets.video_utils import VideoEncodingManager
from lerobot.processor import (
    RobotProcessorPipeline,
    make_default_processors,
    observation_to_transition,
    transition_to_observation,
)
from lerobot.robots.meca500.autocollect.bench import BenchConfig, PlannerLimits
from lerobot.robots.meca500.autocollect.correction import measure
from lerobot.robots.meca500.autocollect.dish_map import DishHeightMap
from lerobot.robots.meca500.autocollect.focus_check import FocusMismatchError, check_focus
from lerobot.robots.meca500.autocollect.overlay import TargetOverlayStep
from lerobot.robots.meca500.autocollect.planner import Planner, PlanningError
from lerobot.robots.meca500.autocollect.stage import MLS203Stage, StageError, StageModel
from lerobot.robots.utils import make_robot_from_config
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted
from lerobot.utils.constants import HF_LEROBOT_HOME
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener, is_headless
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
USER = "AdamAxelrod"
NAME = "basler_touch_corrected10_2026-11-03"  # one dataset per rig and target style - never mix
TASK = "move_pipette_to_target"
NUM_EPISODES = 10
RESUME = False  # append to an existing local dataset of the same NAME
PUSH_TO_HUB = False
DISPLAY_DATA = True

# "hover": touch target raised 5 mm and every speed halved - for watching the first runs.
# "touch": the real thing.
STAGE = "touch"
HOVER_RAISE_MM = 5.0

# Refuse to collect with a calibration worse than this: its residual is the label noise of
# every episode (the gap between marker and where the tip lands).
MAX_CALIBRATION_RESIDUAL_PX = 5.0
# The arm must reach each start within this before recording begins.
START_TOLERANCE_DEG = 0.2
SETTLE_TIMEOUT_S = 30.0
# Before episode 0 the arm drives to HOME_JOINTS (rigs.py) itself. If the tip starts lower than
# this above the dish, the pipette is backed out along its own axis before the joint move.
RETREAT_BELOW_MM = 40.0

# Per-step clamps on the robot side, a backstop above what the planner already enforces
# (motion.max_joint_step_deg / max_joint_step_precision_deg in bench.yaml).
MAX_REL_TARGET = 1.5
PRECISION_MAX_REL_TARGET = 0.15
# Seeds are SEED_BASE + episode index, so a resumed dataset never repeats an episode.
SEED_BASE = 1_000_000

# Vision correction of the final approach (see autocollect/correction.py). Open loop the tip
# landed 60-120 um off the target on 2026-11-03, against a 50 um success threshold.
VISION_CORRECTION = True
CORRECTION_HOLD_S = 0.4  # recorded pause at each checkpoint, so the arm is still when detected
CORRECTION_NEAR_MM = 0.4  # second checkpoint, back along the pipette from the touch
CORRECTION_MID_MM = 1.0  # extra checkpoint, only when the pre-touch one cannot see the tip
CORRECTION_FRAMES = 3  # median of this many frames per detection

# Random dish-stage position per episode (autocollect/stage.py), so the policy never sees the same
# patch of dish twice. Needs the MLS203 (Kinesis closed) and the dish height map auto_calibrate
# writes; each episode touches at the mapped height, and the dish sits in the collision model where
# the stage puts it. The stage moves only while the arm waits at the episode's start pose.
STAGE_MOVES = True
STAGE_ATTEMPTS = 10  # stage positions tried per episode before giving up
MAX_MAP_MISMATCH_MM = 0.05  # the height map must agree with the scope calibration at the reference
# -----------------------------------------------------------------


def build_planner(bench: BenchConfig) -> Planner:
    kin = bench.kinematics()
    calib = bench.calibration()
    if calib.residual_px_rms > MAX_CALIBRATION_RESIDUAL_PX:
        sys.exit(
            f"Scope calibration residual {calib.residual_px_rms:.2f} px > {MAX_CALIBRATION_RESIDUAL_PX}. "
            "Re-run calibrate_scope.py."
        )
    limits = PlannerLimits.from_bench(bench)
    if STAGE == "hover":
        limits = replace(
            limits,
            touch_offset_mm=limits.touch_offset_mm + HOVER_RAISE_MM,
            coarse_speed_mm_s=tuple(v / 2 for v in limits.coarse_speed_mm_s),
            gate_speed_mm_s=tuple(v / 2 for v in limits.gate_speed_mm_s),
            precision_speed_mm_s=tuple(v / 2 for v in limits.precision_speed_mm_s),
            transit_speed_deg_s=limits.transit_speed_deg_s / 2,
        )
    elif STAGE != "touch":
        sys.exit(f"STAGE must be 'hover' or 'touch', not {STAGE!r}")
    return Planner(kin, bench.collision_checker(kin), calib, limits)


def make_ready(meca) -> None:
    """Clear errors, stale queued moves and a resettable safety stop, then check motion runs.

    Closing teleop with Ctrl-C drops its control connection, and the Meca500 answers that
    with a "control connection dropped" safety stop that PAUSES motion until it is resumed.
    Meca500.connect() only clears errors, so without this every MoveJoints is queued behind
    the pause: the arm never moves, the per-step clamp fires on every frame (the controller's
    target never leaves the start pose), and WaitIdle blocks. ResumeMotion() resets the
    resettable stops on firmware >= 10.1 and un-pauses.
    """
    status = meca.GetStatusRobot()
    if status.error_status:
        logging.warning(f"Robot in error ({status.error_code}: {status.error_msg}); resetting.")
        meca.ResetError()
        meca.WaitErrorReset(timeout=10)
    meca.ClearMotion()  # nothing stale from a previous session may execute
    meca.ResumeMotion()
    with contextlib.suppress(Exception):  # the status check below is what decides
        meca.WaitMotionResumed(timeout=5)
    status = meca.GetStatusRobot()
    if status.pause_motion_status or status.error_status:
        safety = meca.GetSafetyStatus()
        raise RuntimeError(
            "The Meca500 is still paused/in error after ResumeMotion - an e-stop or a safety stop "
            f"that cannot be reset remotely is active (stop mask {safety.stop_mask}, resettable "
            f"{safety.stop_resettable_mask}). Release any e-stop, check the web portal, then rerun."
        )
    logging.info("Robot ready: no error, motion resumed, queue empty.")


def wait_settled(robot, target, timeout_s: float) -> np.ndarray:
    """Block until the arm has finished its queued moves; return where it actually is."""
    robot.robot.WaitIdle(timeout=timeout_s)
    q = np.array(robot.robot.GetRtJointPos())
    err = np.abs(q - np.asarray(target)).max()
    if err > START_TOLERANCE_DEG:
        raise RuntimeError(
            f"Arm settled {err:.3f} deg from its planned pose (> {START_TOLERANCE_DEG}); stopping."
        )
    return q


def go_home(robot, teleop, planner, home, events, processors) -> np.ndarray:
    """Drive to ``home`` before anything else, along a collision-checked path.

    If the tip starts low - within RETREAT_BELOW_MM of the dish, e.g. straight after a
    calibration - the pipette is first backed out along its own axis, so a joint-space move
    cannot drag the tip across the dish or the stage opening's edge. Refuses (rather than
    guessing) when no collision-free route home exists.
    """
    q_now = np.array(robot.robot.GetRtJointPos())
    if np.abs(q_now - home).max() < START_TOLERANCE_DEG:
        logging.info("Already at home.")
        return q_now
    tip_height = planner.kin.tip(q_now)[2, 3] - planner.calib.surface_z_mm
    retreat = tip_height < RETREAT_BELOW_MM
    try:
        path = planner.plan_transit(q_now, home, retreat=retreat)
    except PlanningError as e:
        raise RuntimeError(
            f"No collision-free path home from the current pose ({e}). Jog the arm clear of the "
            "microscope with teleoperate_microscope.py, press H, close teleop, then rerun."
        ) from e
    log_say(
        f"Moving home first (tip {tip_height:.0f} mm above the dish"
        f"{', backing out along the pipette' if retreat else ''})",
        False,
    )
    run_plan(robot, teleop, path, np.zeros(len(path)), planner.lim.fps, events, processors)
    return wait_settled(robot, home, SETTLE_TIMEOUT_S)


def run_plan(robot, teleop, joints, precision, fps, events, processors, dataset=None, display=False):
    teleop.load_plan(joints, precision)
    teleop_proc, robot_proc, obs_proc = processors
    record_loop(
        robot=robot,
        events=events,
        fps=fps,
        teleop_action_processor=teleop_proc,
        robot_action_processor=robot_proc,
        robot_observation_processor=obs_proc,
        teleop=teleop,
        dataset=dataset,
        control_time_s=len(joints) / fps,
        single_task=TASK,
        display_data=display,
    )


class EpisodeAbortedError(RuntimeError):
    """The episode cannot be finished as planned (e.g. a correction would collide): discard it."""


def grab(robot, key: str, n: int = CORRECTION_FRAMES) -> np.ndarray:
    """Median of ``n`` raw camera frames (no target overlay: that is an observation processor step)."""
    frames = [robot.get_observation()[key].astype(np.float32) for _ in range(n)]
    return np.median(frames, axis=0).astype(np.uint8)


def run_corrected_episode(robot, teleop, plan, planner, key, background, events, processors, dataset) -> dict:
    """Record one episode in segments, correcting the final approach from the image.

    1. the plan up to the pre-touch point; 2. a still pause, then detect and shift the touch
    point (if the tip is not seen there, one more stop CORRECTION_MID_MM out); 3. a straight move
    to CORRECTION_NEAR_MM before the corrected touch; 4. pause, detect and shift again; 5. the
    touch and the dwell. Then one more (unrecorded) detection measures
    the final error. Returns the correction record for the episode metadata.
    """
    fps = planner.lim.fps
    axis = np.asarray(plan.touch_axis, float)
    hold = max(int(round(CORRECTION_HOLD_S * fps)), 1)
    i = plan.pretouch_index

    def record(joints, precision):
        run_plan(robot, teleop, joints, precision, fps, events, processors, dataset, DISPLAY_DATA)
        if events["rerecord_episode"] or events["stop_recording"]:
            raise EpisodeAbortedError("stopped from the keyboard")

    record(plan.joints[: i + 1], plan.precision[: i + 1])
    q = plan.joints[i]
    p_plan = np.asarray(plan.target_xyz, float)
    p_touch, shift = p_plan, np.zeros(2)
    corrections = []

    def checkpoint(name: str):
        nonlocal shift, p_touch
        record(np.repeat(q[None], hold, axis=0), np.ones(hold))
        expected = planner.calib.xy_to_pixel(planner.kin.tip(q)[:2, 3])[0]  # model tip, now
        c = measure(grab(robot, key), background, expected, planner.calib)
        corrections.append({"checkpoint": name, **c.meta()})
        logging.info(f"  correction at {name}: {c.meta()}")
        # Each checkpoint measures the same model error (detected vs model-predicted tip), so the
        # newest estimate REPLACES the last one - adding them would correct twice. The model stays
        # wrong by the same amount after a shift; a skipped measurement (note set) keeps the last one.
        if not c.note:
            shift = np.asarray(c.shift_mm, float)
        p_touch = p_plan + np.array([*shift, 0.0])
        return c

    def approach(back_mm: float) -> None:
        nonlocal q
        try:
            seg = planner.plan_line(q, p_touch - axis * back_mm, plan.precision_speed_mm_s, "approach")
        except PlanningError as e:
            raise EpisodeAbortedError(str(e)) from e
        record(seg, np.ones(len(seg)))
        q = seg[-1]

    if checkpoint("pretouch").note and plan.pretouch_back_mm > CORRECTION_MID_MM + 0.2:
        # The pre-touch tip was off-screen or not detected. With only the near checkpoint, two of
        # ten touches ended at ~45 um (2026-11-03), so look once more, 1 mm out, where it is in view.
        approach(CORRECTION_MID_MM)
        checkpoint("mid")
    approach(CORRECTION_NEAR_MM)
    checkpoint("near")
    try:
        seg = planner.plan_line(q, p_touch, plan.precision_speed_mm_s, "touch")
    except PlanningError as e:
        raise EpisodeAbortedError(str(e)) from e
    dwell = max(plan.dwell_frames, 1)
    record(np.vstack([seg, np.repeat(seg[-1][None], dwell, axis=0)]), np.ones(len(seg) + dwell))
    final = measure(grab(robot, key), background, plan.target_px, planner.calib)
    um = 1000 * float(np.linalg.norm(np.asarray(planner.calib.px_to_xy)[:, 0]))
    final_um = None if final.error_px is None else final.error_px * um
    logging.info("  final: " + ("tip not detected" if final_um is None else f"{final.error_px:.1f} px = {final_um:.0f} um"))
    return {
        "corrections": corrections,
        "final": final.meta(),
        "final_error_um": None if final_um is None else round(final_um, 1),
        "corrected_target_xyz_mm": [round(float(v), 4) for v in p_touch],
    }


class StagePlanner:
    """Draws the stage position for each episode and keeps the planner in step with the dish.

    ``place(xy)`` puts the dish obstacles where the stage puts them and sets the touch height from
    the dish height map; ``plan_episode`` tries stage positions until one plans.
    """

    def __init__(self, planner: Planner, model: StageModel, hmap: DishHeightMap, base_calib):
        self.planner, self.model, self.hmap, self.base_calib = planner, model, hmap, base_calib

    def place(self, xy) -> float:
        z = self.hmap.surface_z(xy)
        self.planner.checker.set_stage_offset(self.model.dish_offset(xy))
        self.planner.calib = replace(self.base_calib, surface_z_mm=z)
        return z

    def plan_episode(self, seed: int):
        rng = np.random.default_rng(seed + 7_000_000)  # stage draws: separate from the path's
        last = None
        for _ in range(STAGE_ATTEMPTS):
            xy = self.model.sample(rng)
            self.place(xy)
            try:
                return xy, self.planner.sample_episode(seed)
            except PlanningError as e:
                last = e
        raise PlanningError(f"No stage position in {STAGE_ATTEMPTS} tries gave a plan ({last}).")

    def check_parked_arm(self, q, xy_from, xy_to) -> None:
        """The dish sweeps under the parked arm while the stage moves: check every 10%."""
        try:
            for f in np.linspace(0, 1, 11):
                self.planner.checker.set_stage_offset(
                    self.model.dish_offset(np.asarray(xy_from) + f * (np.asarray(xy_to) - np.asarray(xy_from)))
                )
                c = self.planner.checker.clearance(q)
                if c.min_mm < 0:
                    raise PlanningError(f"the stage move would bring the dish into the arm ({c.where} vs {c.obstacle})")
        finally:
            self.planner.checker.set_stage_offset(self.model.dish_offset(xy_from))


def connect_stage(bench, base_calib):
    """Stage driver, model and height map for STAGE_MOVES; refuses on anything stale."""
    path = bench.dish_map_path()
    if not path.exists():
        sys.exit(f"STAGE_MOVES needs {path.name}: run auto_calibrate.py (HEIGHT_MAP = True) first, or set STAGE_MOVES = False.")
    hmap = DishHeightMap.load(path)
    model = StageModel.from_bench(bench.section("scope"), base_calib)
    z_map = hmap.surface_z(model.reference_xy)
    if abs(z_map - base_calib.surface_z_mm) > MAX_MAP_MISMATCH_MM:
        sys.exit(
            f"The dish height map ({z_map:.3f}) disagrees with the scope calibration ({base_calib.surface_z_mm:.3f}) "
            "at the reference stage position: they are from different calibrations. Re-run auto_calibrate.py."
        )
    stage = MLS203Stage(bench.section("scope")["stage_serial"])
    try:
        stage.connect()
    except StageError as e:
        sys.exit(f"{e}\n(Or set STAGE_MOVES = False to record with the stage at its reference position.)")
    logging.info(
        f"Stage moves on: {path.name} tilt {hmap.tilt_deg:.3f} deg; positions within {model.radius_mm} mm of "
        f"{model.centre_xy.tolist()}."
    )
    return stage, model, hmap


def main() -> None:
    init_logging()
    if DISPLAY_DATA:
        # record_loop streams to Rerun when display_data is set; without init the first
        # logged frame raises "No application id found" (as record.py does at startup).
        init_rerun(session_name="record_auto")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    try:
        pos = check_focus(bench.section("scope"))
    except FocusMismatchError as e:
        sys.exit(f"\nREFUSING TO START: {e}\n")
    print(f"Focus check passed: K-Cube at {pos:.4f} mm, as calibrated.")
    planner = build_planner(bench)
    base_calib = planner.calib
    stage = stager = None
    if STAGE_MOVES:
        stage, model, hmap = connect_stage(bench, base_calib)
        stager = StagePlanner(planner, model, hmap, base_calib)
    home = np.asarray(require_home_joints(RIG), dtype=float)
    fps = planner.lim.fps
    repo_id = f"{USER}/{NAME}"

    robot = make_robot_from_config(
        rig_config(
            RIG,
            monitor_mode=False,
            relative_actions=False,
            max_relative_target=MAX_REL_TARGET,
            precision_max_relative_target=PRECISION_MAX_REL_TARGET,
        )
    )
    teleop = Meca500Scripted(Meca500ScriptedConfig(id="meca500_scripted"))

    overlay = TargetOverlayStep(camera_key=bench.camera_key, style=bench.target_style())
    teleop_proc, robot_proc, _ = make_default_processors()
    obs_proc = RobotProcessorPipeline(
        steps=[overlay], to_transition=observation_to_transition, to_output=transition_to_observation
    )
    processors = (teleop_proc, robot_proc, obs_proc)

    features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_proc,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=True,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=obs_proc,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=True,
        ),
    )
    root = HF_LEROBOT_HOME / repo_id
    if RESUME:
        dataset = LeRobotDataset.resume(repo_id, root=root, image_writer_threads=4)
        sanity_check_dataset_robot_compatibility(dataset, robot, fps, features)
    else:
        if root.exists():
            info = root / "meta" / "info.json"
            n_saved = (
                json.loads(info.read_text(encoding="utf-8")).get("total_episodes", 0) if info.exists() else 0
            )
            if n_saved:
                sys.exit(
                    f"{root} already holds {n_saved} episodes. Set RESUME = True to append, or choose a new NAME."
                )
            # Left by a run that failed before saving anything (e.g. the arm was paused):
            # nothing to lose, and refusing would force a manual delete before every retry.
            logging.warning(f"Removing {root}: left by an earlier run with 0 saved episodes.")
            shutil.rmtree(root)
        sanity_check_dataset_name(repo_id, None)
        dataset = LeRobotDataset.create(
            repo_id, fps, robot_type=robot.name, features=features, use_videos=True, image_writer_threads=4
        )
    prov = root / "autocollect"
    prov.mkdir(exist_ok=True)
    shutil.copy(bench.root / "bench.yaml", prov / f"bench_{STAGE}.yaml")
    shutil.copy(bench.calibration_path(), prov / "scope_calibration.json")
    meta_path = root / "autocollect_episodes.jsonl"

    robot.connect()
    robot.configure()
    teleop.connect()
    listener, events = init_keyboard_listener()
    try:
        make_ready(robot.robot)
        stage_xy = None
        if stager is not None:
            if not stage.is_homed:
                # Homing sweeps the dish to the middle of the travel: only with the arm at home.
                if np.abs(np.array(robot.robot.GetRtJointPos()) - home).max() > START_TOLERANCE_DEG:
                    sys.exit("The stage is not homed. Home it in Kinesis with the arm at home (or rerun with the arm at home).")
                stage.home()
            stage_xy = stage.position()
            stager.place(stage_xy)
        q_now = go_home(robot, teleop, planner, home, events, processors)
        retreat = False  # the first episode's transit starts from home, not the dish
        with VideoEncodingManager(dataset):
            recorded = 0
            while recorded < NUM_EPISODES and not events["stop_recording"]:
                ep = dataset.num_episodes
                try:
                    if stager is None:
                        plan = planner.sample_episode(SEED_BASE + ep)
                        new_xy = None
                    else:
                        new_xy, plan = stager.plan_episode(SEED_BASE + ep)
                        # The transit runs before the stage moves: plan it against the dish where it is.
                        stager.place(stage_xy)
                        stager.check_parked_arm(plan.start_joints, stage_xy, new_xy)
                    transit = planner.plan_transit(q_now, plan.start_joints, retreat=retreat)
                except PlanningError as e:
                    logging.error(str(e))
                    break

                overlay.set_target(None)
                log_say(f"Moving to start of episode {ep}", False)
                run_plan(robot, teleop, transit, np.zeros(len(transit)), fps, events, processors)
                wait_settled(robot, plan.start_joints, SETTLE_TIMEOUT_S)
                if events["stop_recording"]:
                    break

                if stager is not None:
                    log_say(f"Moving the dish to stage {np.round(new_xy, 2).tolist()}", False)
                    stage_xy = stage.move_to(new_xy)
                    surface_z = stager.place(stage_xy)
                    extra_stage = {"stage_xy_mm": [round(float(v), 4) for v in stage_xy], "surface_z_mm": round(surface_z, 4)}
                else:
                    extra_stage = {}
                # Background for the tip detector: the start pose keeps the tip out of view.
                background = grab(robot, bench.camera_key) if VISION_CORRECTION else None
                overlay.set_target(plan.target_px)
                log_say(f"Recording episode {ep}", False)
                events["exit_early"] = False
                t0 = time.perf_counter()
                extra: dict = {}
                aborted = None
                if VISION_CORRECTION:
                    try:
                        extra = run_corrected_episode(
                            robot, teleop, plan, planner, bench.camera_key, background, events, processors, dataset
                        )
                    except EpisodeAbortedError as e:
                        aborted = str(e)
                else:
                    run_plan(
                        robot, teleop, plan.joints, plan.precision, fps, events, processors, dataset, DISPLAY_DATA
                    )
                q_now = np.array(robot.robot.GetRtJointPos())
                retreat = True
                if aborted is not None:
                    dataset.clear_episode_buffer()
                    events["rerecord_episode"] = events["exit_early"] = False
                    logging.warning(f"Episode {ep} discarded: {aborted}")
                    continue

                if robot.robot.GetStatusRobot().error_status:
                    raise RuntimeError("Robot reported an error during the episode; stopping.")
                if events["rerecord_episode"] or events["exit_early"] or events["stop_recording"]:
                    # A cut-short scripted episode is not a demonstration - it never reached
                    # the target - so it is discarded rather than saved.
                    events["rerecord_episode"] = events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    logging.info(f"Episode {ep} discarded.")
                    continue

                n_frames = int(dataset.writer.episode_buffer.get("size", 0))
                dataset.save_episode()
                recorded += 1
                with meta_path.open("a") as f:
                    # n_frames: what the dataset holds (the plan's own count can differ by a frame or
                    # two, because record_loop paces by wall time).
                    record_meta = {"episode_index": ep, "stage": STAGE, **plan.meta(), **extra_stage, **extra}
                    record_meta["planned_frames"], record_meta["n_frames"] = record_meta["n_frames"], n_frames
                    f.write(json.dumps(record_meta) + "\n")
                logging.info(
                    f"episode {ep}: {plan.duration_s:.1f}s planned, {time.perf_counter() - t0:.1f}s wall, "
                    f"target px {np.round(plan.target_px, 1).tolist()}, clearance {plan.clearance.min_mm:.1f} mm"
                )

        # Never finish parked over the dish: back the pipette out and go home before the
        # disconnect below (which now holds the pose rather than going limp - but home is
        # where the next session expects to start). Esc sets exit_early too; clear it so
        # this move is not cut short.
        events["exit_early"] = False
        try:
            go_home(robot, teleop, planner, home, events, processors)
        except RuntimeError as e:
            logging.error(f"Could not return home ({e}); the arm holds its current pose.")

        if PUSH_TO_HUB:
            dataset.push_to_hub(private=True)
    finally:
        overlay.set_target(None)
        with contextlib.suppress(Exception):
            robot.disconnect()
        with contextlib.suppress(Exception):
            teleop.disconnect()
        if stage is not None:
            with contextlib.suppress(Exception):
                stage.disconnect()
        if not is_headless() and listener is not None:
            with contextlib.suppress(Exception):
                listener.stop()


if __name__ == "__main__":
    main()
