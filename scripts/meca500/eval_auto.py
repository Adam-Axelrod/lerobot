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
               policy_asr - MODEL brings the tip under the microscope while Huanyu's tip detector
                          (ActionStateRuntime, autocollect/asr_bridge.py) watches every frame; as
                          soon as it sees the tip (WATCH_HITS confident frames in a row) the policy
                          stops and a classical controller finishes: lower the tip straight down
                          to SERVO_HEIGHT_MM, ASR's visual servo onto the target pixel, then a
                          straight-down touch. If the policy ends without the tip in view, SEARCH
                          looks for it: a bounded spiral at a safe height around where the policy
                          left the tip, every move collision-checked. Episodes that needed the
                          search are marked ("search_used"), so results are reported with and
                          without it. Each stage is logged with its time and outcome.
               scripted_asr - the baseline for policy_asr: the planned path (aimed at the target
                          through the calibration) to its pre-touch point, then the same stages.
    4. measure - final tip position from forward kinematics AND from the tip detector on the
               raw frame, both against the target, in px and um; raw frames are saved for the
               by-eye check, plus the joint trajectory for the smoothness metric.

The robot runs with relative_actions=False throughout: this script integrates the policy's
delta onto the controller's current target itself, so that it can check the resulting pose
before sending it. Transits are absolute scripted moves, as in record_auto.

BASE_OFFSET_MM slides the robot base in software (autocollect/asr_bridge.VirtualSlide), for the
"move the robot relative to the microscope" test: the policy is shown, and commands, the joints of
a robot whose base sits BASE_OFFSET_MM away, while the real arm puts the tip where that robot
would. The start pose moves with it. Every method also aims through the scope map shifted by
the offset (autocollect/slide.SoftwareSlideBench), as the pinned pre-slide map would after a
physical slide, so scripted and expert commands land the offset off too. Images, contact,
heights and collision checks are all real. Rows carry slide.software_slide = true.

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
from dataclasses import replace as dc_replace
import json
import logging
import re
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
from lerobot.robots.meca500.autocollect.asr_bridge import (
    ProcessTipDetector,
    Meca500AsrHost,
    TipWatcher,
    VirtualSlide,
    initialize_in_this_thread,
    make_service,
)
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.slide import software_slide_benches, split_benches
from lerobot.robots.meca500.autocollect.disturbance import sample_shift, shift_frame
from lerobot.robots.meca500.autocollect.focus_check import FocusMismatchError, check_focus
from lerobot.robots.meca500.autocollect.operator_guard import OperatorGuard
from lerobot.robots.meca500.autocollect.overlay import TargetCropStep, TargetOverlayStep, draw_target
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
# Which row this session fills: "policy", "policy_asr", "scripted", "scripted_asr", "expert" or "human"
# (see the docstring).
METHOD = "human"
# Pixels to shift every frame by (12.85 um/px), simulating a knocked camera. 0 is a normal run.
# 16 px (~0.2 mm) is inside the expert's 0.5 mm correction cap; 58 px (~0.75 mm) is beyond it.
IMAGE_SHIFT_PX = 0
# The software slide of the robot base (robot frame, mm; x, y in the table plane). (0, 0) is a
# normal run. Every method: autocollect/slide.SoftwareSlideBench.
BASE_OFFSET_MM = (0.0, 0.0)
# A later session carrying on after a stop: runs seeds SEED_BASE + FIRST_EPISODE onwards.
FIRST_EPISODE = 0
# A local pretrained_model folder or a Hub model repo (downloaded once into the HF cache).
# Must be trained on a DELTA dataset recorded on this rig with the digital target.
# Local copies come from CREATE by scp: the lab PC downloads from the Hub at ~100 KB/s.
MODEL = "outputs/train/basler_stage_touch_decoyrec_2026-11_delta/checkpoints/100000/pretrained_model"
EVAL_NAME = "basler_stage_touch_decoyrec_2026-11_delta_act"  # names the "policy" results folder; see session_name()
TASK = "move_pipette_to_target"
NUM_EPISODES = 20  # experienced operator (Adam), 2026-11-08; practice ran as OPERATOR "practice" on SEED_BASE 5_900_000
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
# Chunk mode only. An int re-plans after this many steps during the coarse approach (before the
# policy first sets the precision flag), instead of running the whole chunk open-loop. Each
# 30-step coarse chunk carries the tip 3-9 mm blind, and the 2026-11-05 runs reached precision
# entry aimed 2-3 mm off (bench_error_timeline.py). A chunk that holds still at any point runs
# to its end, so the pre-touch holds do not stall as they did with ensembling. None = off.
COARSE_ACTION_STEPS: int | None = None
# "policy" only: the planner drives each episode to its pre-touch point exactly as the expert
# does (path aimed at a decoy, same collision checks and stage move, no vision correction), the
# arm settles, and the policy takes over from there. Tests whether the learned final approach
# converges from a start inside its training range, apart from the approach (2026-11-06: A and
# the no-decoy model reached precision entry aimed 2.1-2.7 mm off, the demos 0.18-0.67 mm).
# Time, the trajectory log and the scores count from the handover. The settle rule cannot end an
# episode until the tip has moved HANDOVER_MIN_MOVE_UM, since the demos hold still on arrival.
HANDOVER = False
HANDOVER_MIN_MOVE_UM = 50.0
# "policy" only: start each episode the way the finish-only demos started (record_auto
# RECOVERY_KIND "finish", basler_stage_touch_finish_2026-11): the planner brings the tip into view
# 0.1-1.5 mm beside and 0.5-1.5 mm above the target (planner.plan_finish, an unrecorded lead-in),
# the arm settles, and the policy takes over. Tests a finish-only policy on its own; timing and the
# settle rule as HANDOVER. Seeds from FINISH_SEED_BASE (the training batch used 4_000_000 up).
FINISH_START = False
FINISH_SEED_BASE = 5_600_000
# FINISH_START, tip-action policies: "policy" uses the policy's own tip_z; "scripted" uses only its
# tip_u/v while the tip descends straight to the planned touch height at FINISH_DESCENT_MM_S (an
# image-only policy cannot see its height: model E hovered 1.1-1.5 mm up, 2026-11-06).
FINISH_Z = "policy"
FINISH_DESCENT_MM_S = 0.15
# ACT chunk mode: re-plan after this many steps instead of the trained n_action_steps (30 = 1.5 s
# blind), so the image is read more often. None = as trained.
N_ACTION_STEPS: int | None = None

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
# Every VIDEO_EVERY-th frame read during the drive (20 Hz in the policy loop and scripted moves),
# kept at half size and written as epNNN/episode.mp4 with the target drawn on. 0 = no video.
VIDEO_EVERY = 2

# --- METHOD = "policy_asr" ---
# ASR's site file: perception package (ViT-B ONNX), image size and the initial pixel/robot Jacobian
# from scope_calibration.json. Needs the ActionStateRuntime clone installed in .venv.
ASR_SITE_YAML = "assets/benches/meca500_basler/asr_site.yaml"
# The servo stops within this many px of the target (13.3 um/px). ASR's own default is 30 px.
ASR_CONVERGE_PX = 3.0
ASR_TIMEOUT_S = 120.0
# ASR's anchor must lie within this of the tip our detector process saw just before (both are ASR's
# own tip convention, so no offset): a farther anchor is on something else.
ANCHOR_MAX_PX = 30.0
# ASR's tip point is not the pipette's free end: it sits back along the shaft (2026-11-07 bench:
# the servo converged on ASR's point and left the free end ~340 um short). The target handed to
# ASR is the ring plus this offset (ASR px - free-end px at the servo height), measured by
# scripts/meca500/asr_tip_offset.py into the bench's asr_tip_offset.json. Required for *_asr.
ASR_TIP_OFFSET_FILE = "assets/benches/meca500_basler/asr_tip_offset.json"
ASR_TIP_OFFSET_PX = (0.0, 0.0)  # set from ASR_TIP_OFFSET_FILE in main()
# Handover: this many detections in a row at or above this confidence, within WATCH_JUMP_PX of
# each other. On the 2026-11-05/06 rollout frames the tip scored 0.8-1.0 once in view; the only
# stray detections (single frames, tip out of view) scored 0.63-0.68.
WATCH_MIN_CONFIDENCE = 0.8
WATCH_HITS = 2
WATCH_JUMP_PX = 120.0
# The fallback when the policy ends with the tip out of view. False reports the "without search"
# row directly; with True the episodes that searched are marked, so both rows come from one run.
SEARCH = True
SEARCH_HEIGHT_MM = 1.0  # the search runs this far above the dish (straight up or down to it first)
SEARCH_STEP_MM = 1.0  # spacing of the spiral's rings and points (the lit field is 4.8 mm across)
SEARCH_RADIUS_MM = 4.0
SEARCH_SPEED_MM_S = 2.0
# The servo runs with the tip this far above the dish: the demos first reach the ring in the image
# ~0.1 mm up, and the tip stays sharp and detectable there. Then a vertical touch.
SERVO_HEIGHT_MM = 0.15
DESCENT_SPEED_MM_S = 1.0
TOUCH_SPEED_MM_S = 0.2
# Bounds on the servo's moves, which are level (ASR's Jacobian has no z column), straight and
# collision-checked: the longest single move, and how far from where it started the servo may take
# the tip. The lit field is 4.8 mm across, and ASR covered 2 mm in one move in the 2026-11-06 dry run.
ASR_MAX_STEP_MM = 5.0
ASR_MAX_TRAVEL_MM = 5.0

# --- METHOD = "human" ---
OPERATOR = "experienced"  # names the results folder: "experienced", "novice1", "embryology", ...
HUMAN_TIME_S = 120.0  # as in record_microscope.py
# The SpaceMouse gains of teleoperate_microscope.py and record_microscope.py.
GAIN_TR = 50.0
GAIN_TR_FINE = 5.0
# -----------------------------------------------------------------


def safety_surface(planner) -> float:
    """The dish surface (robot-frame z) under the press limit and the floor of every policy_asr
    move: the current dish height map's, for every method and after a physical slide too (the
    methods keep only the pre-slide pixel <-> XY map, autocollect/slide.py)."""
    return planner.calib.surface_z_mm


def deaf() -> dict:
    """Events for the scripted moves between episodes, NOT wired to the keyboard. The listener is
    global, so arrow keys pressed in any window cut transits short twice on 2026-11-05 (16.8 and
    25.2 deg from the start pose). Esc still stops the session between episodes."""
    return {"exit_early": False, "rerecord_episode": False, "stop_recording": False}


def session_name() -> str:
    name = {"policy": EVAL_NAME, "human": f"human_{OPERATOR}"}.get(METHOD, METHOD)
    if METHOD == "policy" and IGNORE_PRECISION:
        name += "_precision_off"
    if METHOD == "policy" and COARSE_ACTION_STEPS:
        name += f"_coarse{COARSE_ACTION_STEPS}"
    if METHOD == "policy" and HANDOVER:
        name += "_handover"
    if METHOD == "policy" and FINISH_START:
        name += "_finish" + ("_zscripted" if FINISH_Z == "scripted" else "")
    if METHOD == "policy" and N_ACTION_STEPS and TEMPORAL_ENSEMBLE_COEFF is None:
        name += f"_n{N_ACTION_STEPS}"
    if METHOD == "policy_asr":
        name = f"{EVAL_NAME}_asr" + ("" if SEARCH else "_nosearch")
    if METHOD == "scripted_asr":
        name += "" if SEARCH else "_nosearch"
    if any(BASE_OFFSET_MM):
        name += "_swslide" + "x".join(f"{v:g}" for v in BASE_OFFSET_MM)
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
        # The newest frame any reader got, for the tip watcher's thread (which must not read the
        # robot itself while the control loop does).
        self.latest: np.ndarray | None = None
        self.heartbeat: Path | None = None
        self._beat = 0.0
        # Every video_every-th frame read while started, at half size, for epNNN/episode.mp4.
        self.video_every = 0
        self.video: list[np.ndarray] = []
        robot.get_observation = self.read

    def read(self) -> dict:
        obs = self._read()
        if self.shift != (0, 0):
            obs[self._key] = shift_frame(obs[self._key], *self.shift)
        self.latest = obs[self._key]
        now = time.perf_counter()
        if self.heartbeat is not None and now - self._beat >= 1.0:
            # Liveness for scripts/meca500/stall_watch.py: every reader passes here, ~20 Hz.
            self._beat = now
            with contextlib.suppress(OSError):
                self.heartbeat.write_text(f"{time.time():.1f}\n", encoding="utf-8")
        if self._t is not None and self.video_every and len(self._t) % self.video_every == 0:
            f = obs[self._key]
            self.video.append((time.perf_counter() - self._t0, np.ascontiguousarray(f[::2, ::2])))
        if self._t is not None:
            self._t.append(time.perf_counter() - self._t0)
            self._q.append([float(obs[f"joint_{i + 1}.pos"]) for i in range(6)])
        return obs

    def start(self) -> None:
        self._t0 = time.perf_counter()
        self._t, self._q = [], []
        self.video = []

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
    elif cfg.type == "act" and N_ACTION_STEPS:
        cfg.n_action_steps = N_ACTION_STEPS
    train_cfg = json.loads((path / "train_config.json").read_text(encoding="utf-8"))
    # add_slide_basis.py's copies (_sb) add a training-only column to the same delta dataset.
    dataset = train_cfg["dataset"]["repo_id"].removesuffix("_sb")
    if not dataset.endswith("_delta"):
        sys.exit(f"{model} was trained on {dataset}, not a delta dataset: this script sends deltas.")
    policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
    policy.eval()
    # Image-only policies trained on camera-axis tip motion (convert_to_tip_actions.py); IK turns
    # each action into a joint target in rollout_episode.
    policy.tip_actions = dataset.endswith("_tip_delta")
    # Trained on crop_dataset.py's crops around the target: the live frame gets the same crop.
    m = re.search(r"_crop(\d+)_", dataset)
    policy.crop_px = int(m.group(1)) if m else None
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=str(path), preprocessor_overrides={"device_processor": {"device": DEVICE}}
    )
    logging.info(f"Policy {cfg.type} from {path} (trained on {dataset}, {'tip' if policy.tip_actions else 'joint'} actions)")
    return policy, pre, post


def rollout_episode(
    robot, policy, pre, post, obs_proc, obs_features, planner, events, live_key: str | None = None,
    min_move_um: float = 0.0, stop_when=None, slide: VirtualSlide | None = None, z_touch: float | None = None,
) -> dict:
    """Run the policy until it settles, the time runs out, a check fails or the operator stops it.

    With ``live_key``, the result's "live" holds (step, frame, joints) at each policy call: the
    camera frame before the marker overlay, copied in memory (see SAVE_LIVE_FRAMES).
    ``stop_when`` (a threading.Event) ends the episode as soon as it is set: the handover.
    ``slide`` runs the policy on a virtually displaced base: it sees and commands the virtual
    joints, the arm gets the real ones (BASE_OFFSET_MM). ``z_touch`` (tip actions, FINISH_Z
    "scripted"): the policy's tip_z is replaced by a straight descent to this height (mm)."""
    kin, checker, calib = planner.kin, planner.checker, planner.calib
    fps = planner.lim.fps
    policy.reset()
    pre.reset()
    post.reset()
    log = {k: [] for k in ("t", "q_cmd", "q_meas", "delta", "precision", "clamped", "backstop_cut")}
    n_steps = int(getattr(getattr(policy, "config", None), "n_action_steps", 1))
    live_every = n_steps if n_steps > 1 else LIVE_EVERY_ENSEMBLE
    live = []
    reason, settled, ever_precise = "time limit", 0, False
    chunk_steps, chunk_held = 0, False
    first_tip = None
    prev_tip = None
    # Integrate onto the last goal SENT, not the controller's live setpoint: the training deltas
    # are differences between successive commanded goals, and GetRtTargetJointPos() lags the
    # goal while a move is in flight. Integrating onto it dropped the unfinished part of every
    # step - the first bench test (2026-11-05) commanded only 54% of what the policy asked for
    # and never reached the dish. The arm is at rest after the transit, so the setpoint is the goal.
    q_goal = np.asarray(robot.robot.GetRtTargetJointPos(), dtype=float)
    if slide is not None:
        q_goal = slide.to_virtual(q_goal)  # the goal in the policy's (virtual) frame
        if q_goal is None:
            raise PlanningError("no IK solution for the virtual start pose")
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
        if stop_when is not None and stop_when.is_set():
            reason = "handover: tip in view"
            break
        raw = robot.get_observation()
        if slide is not None:
            q_virtual = slide.to_virtual([raw[f"joint_{j + 1}.pos"] for j in range(6)])
            if q_virtual is None:
                reason = "stopped: no IK solution for the virtual joint state"
                break
            raw = {**raw, **{f"joint_{j + 1}.pos": float(q_virtual[j]) for j in range(6)}}
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
        if COARSE_ACTION_STEPS and TEMPORAL_ENSEMBLE_COEFF is None:
            chunk_steps += 1
            chunk_held |= bool(np.abs(a[:6]).max() < 1e-4)
            if not policy._action_queue:
                chunk_steps, chunk_held = 0, False
            elif chunk_steps >= COARSE_ACTION_STEPS and not ever_precise and not chunk_held:
                policy._action_queue.clear()  # the next step predicts a fresh chunk
                chunk_steps = 0
        normal_clamp = PRECISION_MAX_REL_TARGET if precise else MAX_REL_TARGET
        clamp = MAX_REL_TARGET if IGNORE_PRECISION else normal_clamp
        if tip_mode:
            if z_touch is not None:
                a = a.copy()
                a[2] = -min(FINISH_DESCENT_MM_S / fps, max(float(t_goal[2, 3]) - z_touch, 0.0))
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
        q_policy = q_cmd
        if slide is not None:
            q_cmd = slide.to_real(q_policy)
            if q_cmd is None:
                reason = "stopped: no IK solution for the slid command"
                break

        tip = kin.tip(q_cmd)[:3, 3]
        height = float(tip[2] - safety_surface(planner))
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
        q_sent = np.array([sent[f"joint_{i + 1}.pos"] for i in range(6)], dtype=float)
        if slide is None:
            q_goal = q_sent
        else:
            cut = np.abs(q_sent - q_cmd).max() > 1e-6
            q_goal = slide.to_virtual(q_sent) if cut else q_policy
            if q_goal is None:
                reason = "stopped: no IK solution for the virtual goal"
                break
        if tip_mode:
            # A clamped step did not reach t_next: carry on from where the goal really is.
            t_goal = t_next if np.abs(q_goal - q_ik).max() < 1e-6 else kin.tip(q_goal)
        log["backstop_cut"].append(bool(np.abs(q_sent - q_cmd).max() > 1e-6))
        log["t"].append(t0 - t_start)
        log["q_cmd"].append(q_cmd)
        log["q_meas"].append(np.asarray(robot.robot.GetRtJointPos(), dtype=float))
        log["delta"].append(a[:6])
        log["precision"].append(precise)
        # Against the normal clamp in every run, so "precision off" shows what the flag would have done.
        log["clamped"].append(bool(np.any(np.abs(joint_step) > normal_clamp)))

        step_um = 0.0 if prev_tip is None else float(np.linalg.norm(tip - prev_tip)) * 1000
        prev_tip = tip
        first_tip = tip if first_tip is None else first_tip
        moved = float(np.linalg.norm(tip - first_tip)) * 1000 >= min_move_um
        settled = settled + 1 if (ever_precise and moved and step_um < SETTLED_UM_PER_STEP) else 0
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


def save_video(ep_dir: Path, frames: list, target, style, fps: float) -> None:
    """epNNN/episode.mp4: the kept (time, frame) pairs back at full size, with the target drawn on;
    episode_t.json holds each frame's time since the drive started (the trajectory's clock)."""
    if not frames:
        return
    import cv2

    h, w = frames[0][1].shape[0] * 2, frames[0][1].shape[1] * 2
    ep_dir.mkdir(parents=True, exist_ok=True)
    out = cv2.VideoWriter(str(ep_dir / "episode.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for _, f in frames:
        full = cv2.resize(f, (w, h), interpolation=cv2.INTER_LINEAR)
        out.write(cv2.cvtColor(draw_target(full, target, style), cv2.COLOR_RGB2BGR))
    out.release()
    (ep_dir / "episode_t.json").write_text(json.dumps([round(t, 3) for t, _ in frames]), encoding="utf-8", newline="\n")


def measure_final(robot, arm, planner, bench, background, target_px, shift=(0, 0), calib=None) -> tuple[dict, np.ndarray]:
    """Tip vs target by forward kinematics and by the tip detector, after the arm stops.

    ``robot`` supplies the frames and ``arm`` the controller connection (they differ for "human").
    ``calib`` is the TRUE calibration (post-slide, see autocollect/slide.py); default the planner's.
    """
    kin, calib = planner.kin, (calib or planner.calib)
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


class StageFailedError(RuntimeError):
    """A policy_asr stage could not be carried out; the arm holds where it is."""


def move_tip_to(robot, teleop, planner, p, speed_mm_s, name, fps, processors, coarse=False) -> np.ndarray:
    """A straight, collision-checked tip move holding the orientation; returns the settled joints."""
    q = np.asarray(robot.robot.GetRtJointPos(), dtype=float)
    floor = safety_surface(planner) - PRESS_LIMIT_MM
    if p[2] < floor:
        # A backstop: no policy_asr move aims below the dish (touch_offset_mm >= 0 keeps them above).
        logging.warning(f"{name}: target {1000 * (floor - p[2]):.0f} um below the press limit; held at the limit.")
        p = np.array([p[0], p[1], floor])
    if np.linalg.norm(planner.kin.tip(q)[:3, 3] - p) < 1e-3:
        return q
    try:
        path = planner.plan_line(
            q, p, speed_mm_s, name, max_step_deg=planner.lim.max_joint_step_deg if coarse else None
        )
    except PlanningError as e:
        raise StageFailedError(str(e)) from e
    # Flag 0: the plan keeps its own step limit, and the policy-mode precision backstop is tighter.
    run_plan(robot, teleop, path, np.zeros(len(path)), fps, deaf(), processors)
    return wait_settled(robot, path[-1], SETTLE_TIMEOUT_S)


def spiral_offsets(step_mm: float, radius_mm: float) -> list[np.ndarray]:
    """Points on rings step_mm apart out to radius_mm, about step_mm apart along each ring."""
    pts = []
    for r in np.arange(step_mm, radius_mm + 1e-9, step_mm):
        n = max(6, int(np.ceil(2 * np.pi * r / step_mm)))
        pts += [r * np.array([np.cos(a), np.sin(a)]) for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
    return pts


def look_for_tip(robot, detector, key: str):
    """Two still frames: the tip counts as found only if both see it, close together."""
    hits = []
    for _ in range(2):
        s = detector.detect(grab(robot, key, 3))
        if s is None or s.confidence < WATCH_MIN_CONFIDENCE:
            return None
        hits.append(s)
    return hits[-1] if np.hypot(*np.subtract(hits[0].px, hits[1].px)) <= WATCH_JUMP_PX else None


def search_for_tip(robot, teleop, planner, detector, key, fps, processors) -> tuple:
    """The fallback: a spiral at a safe height around where the policy left the tip."""
    t0 = time.perf_counter()
    pose = planner.kin.tip(np.asarray(robot.robot.GetRtJointPos(), dtype=float))
    # Always at SEARCH_HEIGHT_MM: much higher and the tip is too blurred to detect (the 2026-11-06
    # handover frames: detected at 0.9-1.0 confidence up to ~1.5 mm above the dish). It gets there
    # along the pipette's own axis - the way every approach comes in - since straight down from a
    # pre-touch point brought the tilted pipette within margin of the dish wall.
    z = planner.calib.surface_z_mm + SEARCH_HEIGHT_MM
    axis = pose[:3, 2]
    p0 = pose[:3, 3] + (z - pose[2, 3]) / axis[2] * axis
    info = {"points_tried": 0, "points_refused": 0, "height_change_mm": round(float(z - pose[2, 3]), 3)}
    move_tip_to(robot, teleop, planner, p0, SEARCH_SPEED_MM_S, "search height", fps, processors, coarse=True)
    found = look_for_tip(robot, detector, key)
    for off in spiral_offsets(SEARCH_STEP_MM, SEARCH_RADIUS_MM):
        if found is not None:
            break
        info["points_tried"] += 1
        try:
            move_tip_to(
                robot, teleop, planner, np.array([p0[0] + off[0], p0[1] + off[1], z]),
                SEARCH_SPEED_MM_S, "search", fps, processors, coarse=True,
            )
        except StageFailedError:
            info["points_refused"] += 1
            continue
        found = look_for_tip(robot, detector, key)
    info["found"] = found is not None
    info["duration_s"] = round(time.perf_counter() - t0, 2)
    return found, info


def run_policy_asr(
    robot, teleop, policy, pre, post, obs_proc, obs_features, planner, events, key, target,
    watcher, detector, service, host, fps, processors, slide, live_key, plan=None, background=None,
) -> dict:
    """METHOD "policy_asr": the learned approach, the handover, then the classical finish.

    With ``policy`` None ("scripted_asr", the baseline) the approach is instead the planned path
    to its pre-touch point, open loop, and the same stages follow."""
    t0 = time.perf_counter()
    kin = planner.kin
    out = {"stages": [], "search_used": False, "failure_stage": None}

    def stage(name: str, t_start: float, **info) -> None:
        out["stages"].append({"stage": name, "t_s": round(t_start - t0, 2), "duration_s": round(time.perf_counter() - t_start, 2), **info})
        out["stage_reached"] = name

    def tip_now() -> np.ndarray:
        return kin.tip(np.asarray(robot.robot.GetRtJointPos(), dtype=float))[:3, 3]

    # 1. The approach: scripted (the baseline), or learned with the detector watching.
    t = time.perf_counter()
    if policy is None:
        k = plan.pretouch_index
        run_plan(robot, teleop, plan.joints[: k + 1], np.zeros(k + 1), fps, events, processors)
        events["exit_early"] = False
        wait_settled(robot, plan.joints[k], SETTLE_TIMEOUT_S)
        out["log"], out["live"] = {}, []
        stage("approach", t, reason="scripted path to the pre-touch point")
        out["reason"] = "scripted"
        watcher.sighting = None
        if events["stop_recording"]:
            return out
        return finish_with_asr(
            robot, teleop, planner, events, key, target, None, detector, service, host, fps, processors, out, t0,
            stage, tip_now, background,
        )
    watcher.start()
    try:
        run = rollout_episode(
            robot, policy, pre, post, obs_proc, obs_features, planner, events,
            live_key=live_key, stop_when=watcher.seen, slide=slide,
        )
    finally:
        watcher.stop()
    robot.robot.WaitIdle(timeout=10)
    out["log"], out["live"] = run.pop("log"), run.pop("live")
    stage("approach", t, reason=run["reason"], loop_hz=run["loop_hz"], detections=watcher.history, watcher_error=watcher.error)
    out["reason"] = run["reason"]
    if events["stop_recording"]:
        return out

    return finish_with_asr(
        robot, teleop, planner, events, key, target, watcher.sighting, detector, service, host, fps, processors,
        out, t0, stage, tip_now, background,
    )


def finish_with_asr(
    robot, teleop, planner, events, key, target, sighting, detector, service, host, fps, processors, out, t0, stage,
    tip_now, background=None,
) -> dict:
    """The classical finish of policy_asr and scripted_asr, from wherever the approach left the tip."""
    from action_state_runtime.contracts import (
        AnchorRequest,
        AnchorRole,
        ExecuteTaskGoal,
        RoutineKind,
        TargetKind,
    )

    um_per_px = planner.calib.mm_per_px * 1000
    # 2. Handover - or the search, if the policy left the tip out of view.
    t = time.perf_counter()
    sighting = sighting or look_for_tip(robot, detector, key)
    if sighting is None:
        if not SEARCH:
            out["failure_stage"] = "view"
            out["reason"] = "tip never in view"
            return out
        out["search_used"] = True
        try:
            sighting, info = search_for_tip(robot, teleop, planner, detector, key, fps, processors)
        except (StageFailedError, RuntimeError) as e:
            sighting, info = None, {"error": str(e)}
        stage("search", t, **info)
        if sighting is None:
            out["failure_stage"] = "search"
            out["reason"] = "search did not find the tip"
            return out
    p = tip_now()
    out["time_to_view_s"] = round(time.perf_counter() - t0, 2) if out["search_used"] else round(sighting.t - t0, 2)
    out["handover"] = {
        "tip_px": [round(v, 1) for v in sighting.px],
        "confidence": round(sighting.confidence, 3),
        "tip_height_um": round(float(p[2] - planner.calib.surface_z_mm) * 1000, 1),
        "vision_to_target_um": round(float(np.hypot(*np.subtract(sighting.px, target))) * um_per_px, 1),
    }
    stage("handover", t, **out["handover"])

    try:
        attempt = "lower"
        # 3. Down (or up) to the servo height. Straight down keeps the tip's image where the
        # handover saw it. Along the pipette's axis instead (2026-11-07 policy_asr x20) moved the tip
        # as far sideways as it dropped - 2-4.5 mm from handovers 2-4.6 mm up - out of the lit field,
        # and ASR's anchor found no tip. The axis is only the fallback for when the collision check
        # refuses the vertical line (near the dish wall, as at a scripted pre-touch point).
        t = time.perf_counter()
        axis = planner.kin.tip(np.asarray(robot.robot.GetRtJointPos(), dtype=float))[:3, 2]
        drop = planner.calib.surface_z_mm + SERVO_HEIGHT_MM - p[2]
        how = "vertical"
        try:
            move_tip_to(robot, teleop, planner, p + [0.0, 0.0, drop], DESCENT_SPEED_MM_S, "lower to servo height", fps, processors)
        except StageFailedError:
            how = "along the pipette (vertical refused)"
            move_tip_to(
                robot, teleop, planner, p + drop / axis[2] * axis, DESCENT_SPEED_MM_S, "lower to servo height", fps, processors
            )
        # The tip must still be in view, as confidently as at the handover, before ASR anchors on it.
        seen = detector.detect(grab(robot, key, 3))
        stage(
            "lower", t, drop_mm=round(float(drop), 3), how=how,
            tip_px_before=[round(v, 1) for v in sighting.px],
            tip_px_after=None if seen is None else [round(v, 1) for v in seen.px],
            confidence_after=None if seen is None else round(seen.confidence, 3),
        )
        if seen is None or seen.confidence < WATCH_MIN_CONFIDENCE:
            out["failure_stage"] = "lower"
            out["reason"] = "tip not in view after lowering"
            return out
        if events["stop_recording"]:
            return out

        attempt = "servo"
        # 4. ASR's visual servo onto the target pixel.
        t = time.perf_counter()
        host.reset()
        anchor = service.capture_anchor(AnchorRequest(AnchorRole.FOCAL), timeout=60.0)
        # The anchor must be on the tip just seen (2026-11-07 ep 8 anchored on a frame-edge point).
        off_tip = anchor.visual_tip_px is not None and np.hypot(*np.subtract(anchor.visual_tip_px, seen.px)) > ANCHOR_MAX_PX
        if not anchor.valid or off_tip:
            why = anchor.reason if not anchor.valid else f"anchor {np.round(anchor.visual_tip_px, 1).tolist()} not on the tip {np.round(seen.px, 1).tolist()}"
            stage("servo", t, anchor=why)
            out["failure_stage"] = "servo"
            out["reason"] = f"ASR anchor failed: {why}"
            return out
        # ASR servos ITS tip point: aim it ASR_TIP_OFFSET_PX from the ring so the free end lands on it.
        asr_target = np.asarray(target, dtype=float) + np.asarray(ASR_TIP_OFFSET_PX, dtype=float)
        handle = service.submit_task(
            ExecuteTaskGoal(target_kind=TargetKind.PIXEL, target_px=tuple(float(v) for v in asr_target), routine=RoutineKind.POSITION)
        )
        if not handle.accepted:
            raise StageFailedError(f"ASR rejected the goal: {handle.rejection_reason}")
        try:
            r = handle.result(timeout=ASR_TIMEOUT_S)
        except Exception:
            handle.cancel()
            r = handle.result(timeout=30.0)
        out["servo"] = {
            "converged": r.converged,
            "reason": r.reason,
            "steps": r.servo_steps,
            "final_tip_px": None if r.final_tip_px is None else [round(v, 1) for v in r.final_tip_px],
            "final_distance_px": None if r.final_distance_px is None else round(float(r.final_distance_px), 2),
            "anchor_tip_px": None if anchor.visual_tip_px is None else [round(v, 1) for v in anchor.visual_tip_px],
            "asr_target_px": [round(float(v), 2) for v in asr_target],
            "tip_offset_px": list(ASR_TIP_OFFSET_PX),
            "moves": host.moves,
        }
        if background is not None:
            # The residual at the servo height, by our own free-end detector (no move): the
            # number reported next to the final distance.
            free = detect_tip(grab(robot, key), background)
            if free is not None:
                out["servo"]["free_end_px"] = [round(float(v), 2) for v in free.px]
                out["servo"]["free_end_to_target_um"] = round(
                    float(np.hypot(*np.subtract(free.px, target))) * planner.calib.mm_per_px * 1000, 1
                )
                out["servo"]["free_end_score"] = round(float(free.score), 2)
        stage("servo", t, converged=r.converged, reason=r.reason, steps=r.servo_steps)
        if not r.converged:
            # No blind touch (2026-11-07 ep 8 touched 6.5 mm off after ASR's first move was refused).
            out["failure_stage"] = "servo"
            out["reason"] = f"ASR did not converge: {r.reason}"
            return out

        attempt = "touch"
        # 5. Straight down onto the dish.
        t = time.perf_counter()
        p = tip_now()
        move_tip_to(
            robot, teleop, planner,
            np.array([p[0], p[1], planner.calib.surface_z_mm + planner.lim.touch_offset_mm]),
            TOUCH_SPEED_MM_S, "touch", fps, processors,
        )
        stage("touch", t)
        out["reason"] = "finished"
    except StageFailedError as e:
        out["failure_stage"] = attempt  # the stage that could not be carried out
        out["reason"] = f"stopped: {e}"
    return out


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
    if METHOD not in ("policy", "policy_asr", "scripted", "scripted_asr", "expert", "human"):
        sys.exit(f"METHOD must be policy, policy_asr, scripted, scripted_asr, expert or human, not {METHOD!r}")
    asr = METHOD in ("policy_asr", "scripted_asr")
    learned = METHOD in ("policy", "policy_asr")
    if asr and (IMAGE_SHIFT_PX or COARSE_ACTION_STEPS or TEMPORAL_ENSEMBLE_COEFF is not None):
        sys.exit("policy_asr / scripted_asr run whole chunks on unshifted frames: set IMAGE_SHIFT_PX 0, COARSE_ACTION_STEPS and ensembling off.")
    human = METHOD == "human"
    record_auto.DISPLAY_DATA = False  # no Rerun stream from run_corrected_episode
    # The test set is normal episodes, whatever record_auto was last set to collect (its
    # StagePlanner plans through record_auto.plan_for).
    record_auto.RECOVERY = False
    if FINISH_START:
        if METHOD != "policy":
            sys.exit("FINISH_START is for METHOD = 'policy'.")
        record_auto.RECOVERY, record_auto.RECOVERY_KIND = True, "finish"
    seed_base = FINISH_SEED_BASE if FINISH_START else SEED_BASE
    if METHOD in ("scripted", "scripted_asr"):
        # Since 2026-11-05 every plan aims at a decoy up to DECOY_MAX_MM off the target (the
        # expert's image correction closes the gap). The scripted controller has no correction, so
        # its plans aim at the true target. A vanishing decoy keeps the decoy-era test set: the
        # true target comes from its own seed stream (planner.plan_with_decoy).
        record_auto.DECOY_MAX_MM = 1e-6
    init_logging()
    register_third_party_plugins()
    try:
        benches = (
            software_slide_benches(BenchConfig.load(RIG), BASE_OFFSET_MM)
            if any(BASE_OFFSET_MM)
            else split_benches(BenchConfig.load(RIG))
        )
    except RuntimeError as e:
        sys.exit(f"\nREFUSING TO START: {e}\n")
    # Every method plans and aims with benches.method: after a physical slide, the pinned pre-slide
    # pixel <-> XY map with the current dish height. benches.truth (the current calibration) is for
    # the final measurement only.
    bench = benches.method
    truth_calib = benches.truth.calibration()
    if benches.measurement is not None:
        logging.info(f"Slide in force: {benches.measurement}. Methods keep the pre-slide pixel <-> XY map.")
    try:
        pos = check_focus(bench.section("scope"))
    except FocusMismatchError as e:
        sys.exit(f"\nREFUSING TO START: {e}\n")
    print(f"Focus check passed: K-Cube at {pos:.4f} mm, as calibrated.")
    policy, pre, post = load_policy(MODEL) if learned else (None, None, None)
    if FINISH_START and FINISH_Z == "scripted" and not policy.tip_actions:
        sys.exit("FINISH_Z 'scripted' needs a tip-action policy (a *_tip_delta dataset).")
    if FINISH_Z not in ("policy", "scripted"):
        sys.exit(f"FINISH_Z must be 'policy' or 'scripted', not {FINISH_Z!r}")

    planner = build_planner(bench)
    stage, model, hmap = connect_stage(bench, planner.calib)
    stager = StagePlanner(planner, model, hmap, planner.calib)
    home = np.asarray(require_home_joints(RIG), dtype=float)
    fps = planner.lim.fps

    # The planned paths keep to the clamps they were recorded under; the policy's are tighter.
    if (
        METHOD == "policy"
        and HANDOVER
        and (COARSE_ACTION_STEPS or TEMPORAL_ENSEMBLE_COEFF is not None or any(BASE_OFFSET_MM))
    ):
        sys.exit("HANDOVER needs whole chunks (COARSE_ACTION_STEPS and ensembling off) and no BASE_OFFSET_MM.")
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
    tap.video_every = VIDEO_EVERY
    target_cfg = bench.section("target")
    overlay = TargetOverlayStep(camera_key=bench.camera_key, style=bench.target_style())
    teleop_proc, robot_proc, _ = make_default_processors()
    obs_proc = RobotProcessorPipeline(
        steps=[overlay] + ([TargetCropStep(overlay=overlay, size=policy.crop_px)] if learned and policy.crop_px else []),
        to_transition=observation_to_transition,
        to_output=transition_to_observation,
    )
    processors = (teleop_proc, robot_proc, obs_proc)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR, use_video=True)
    # The policy also needs the slid robot's joints; every other method aims through the shifted map.
    slide = VirtualSlide(planner.kin, BASE_OFFSET_MM) if any(BASE_OFFSET_MM) and learned else None

    out_dir = Path("outputs/eval_auto") / f"{session_name()}_{time.strftime('%Y%m%d-%H%M%S')}"
    out_dir.mkdir(parents=True)
    tap.heartbeat = out_dir / "heartbeat.txt"
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

    service = host = detector = watcher = None
    if asr:
        global ASR_TIP_OFFSET_PX
        off_path = Path(ASR_TIP_OFFSET_FILE)
        if not off_path.exists():
            sys.exit(f"{off_path} not found: run scripts/meca500/asr_tip_offset.py (ASR's tip point vs the free end) first.")
        off = json.loads(off_path.read_text(encoding="utf-8"))
        ASR_TIP_OFFSET_PX = tuple(float(v) for v in off["offset_px"])
        logging.info(f"ASR tip offset {ASR_TIP_OFFSET_PX} px (n={off.get('n')}, measured {off.get('measured_at')}).")
        key = bench.camera_key
        host = Meca500AsrHost(
            planner,
            frame_source=lambda: grab(robot, key, 3),
            joints=lambda: np.asarray(robot.robot.GetRtJointPos(), dtype=float),
            move=lambda path: (
                run_plan(robot, teleop, path, np.zeros(len(path)), fps, deaf(), processors),
                wait_settled(robot, path[-1], SETTLE_TIMEOUT_S),
            ),
            surface_z_mm=lambda xy: safety_surface(planner),
            max_step_mm=ASR_MAX_STEP_MM,
            max_travel_mm=ASR_MAX_TRAVEL_MM,
            press_limit_mm=PRESS_LIMIT_MM,
        )
        service = make_service(host, ASR_SITE_YAML, out_dir / "asr", converge_px=ASR_CONVERGE_PX)
        logging.info("Loading the ASR perception package...")
        t_load = time.perf_counter()
        # On this thread, not ASR's worker (asr_bridge.initialize_in_this_thread: the worker hung twice).
        initialize_in_this_thread(service)
        if not service.onnx_ready:
            sys.exit("ASR did not initialise.")
        logging.info(f"ASR ready in {time.perf_counter() - t_load:.0f} s.")
        # The detector the watcher and the search use runs in its own process: in this one, it and
        # the policy deadlocked on the GPU, and a lock between them starved the control loop
        # (asr_bridge.ProcessTipDetector). ASR's servo uses the service's own runtime, here, while
        # no policy runs.
        detector = ProcessTipDetector(ASR_SITE_YAML, target_cfg["field_centre_px"], target_cfg["field_radius_px"])
        logging.info("ASR tip detector process ready.")
        watcher = TipWatcher(
            detector, lambda: tap.latest, min_confidence=WATCH_MIN_CONFIDENCE, hits=WATCH_HITS, max_jump_px=WATCH_JUMP_PX
        )

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
                new_xy, plan = stager.plan_episode(seed_base + i)
                start_q = human_start_joints(planner, plan) if human else (plan.lead_in[0] if FINISH_START else plan.start_joints)
                if slide is not None:
                    # The slid robot starts at the planned joints: the real arm starts where that puts the tip.
                    start_q = slide.to_real(start_q)
                    if start_q is None:
                        raise PlanningError(f"episode {i}: no IK solution for the slid start pose")
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
                seed_base + i,
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
            handover_s = None
            if METHOD == "policy" and (HANDOVER or FINISH_START):
                k = plan.pretouch_index
                t_h = time.perf_counter()
                if FINISH_START:  # the lead-in from the high start into view beside the ring
                    path, flags = plan.lead_in, np.zeros(len(plan.lead_in))
                else:
                    path, flags = plan.joints[: k + 1], plan.precision[: k + 1]
                run_plan(robot, teleop, path, flags, fps, events, processors)
                events["exit_early"] = False
                if events["stop_recording"]:
                    break
                wait_settled(controller, path[-1], SETTLE_TIMEOUT_S)
                handover_s = round(time.perf_counter() - t_h, 2)
            log_say(f"{METHOD} episode {i}", False)
            t_drive = time.perf_counter()
            tap.start()
            if asr:
                run = run_policy_asr(
                    robot, teleop, policy, pre, post, obs_proc, obs_features, planner, events,
                    bench.camera_key, target, watcher, detector, service, host, fps, processors, slide,
                    live_key=bench.camera_key if SAVE_LIVE_FRAMES else None, plan=plan, background=background,
                )
            elif METHOD == "policy":
                run = rollout_episode(
                    robot, policy, pre, post, obs_proc, obs_features, planner, events,
                    live_key=bench.camera_key if SAVE_LIVE_FRAMES else None,
                    min_move_um=HANDOVER_MIN_MOVE_UM if HANDOVER or FINISH_START else 0.0,
                    slide=slide,
                    z_touch=plan.target_xyz[2] if FINISH_START and FINISH_Z == "scripted" else None,
                )
                if handover_s is not None:
                    run["handover_s"] = handover_s
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
            m, final = measure_final(
                robot, arm, planner, bench, background, target, shift,
                calib=dc_replace(truth_calib, surface_z_mm=surface_z),
            )
            overlay.set_target(None)
            tap.shift = (0, 0)
            save_raw(ep_dir, "final", final)
            save_raw(ep_dir, "final_overlay", draw_target(final, target, bench.target_style()))
            log = run.pop("log", {})
            np.savez_compressed(ep_dir / "traj.npz", **traj, **{k: np.asarray(v) for k, v in log.items()})
            live = run.pop("live", [])
            save_live(ep_dir, live, target)
            save_video(ep_dir, tap.video, target, bench.target_style(), fps / max(VIDEO_EVERY, 1))
            tap.video = []

            row = {
                "episode": i,
                "seed": seed_base + i,
                "method": METHOD,
                "operator": OPERATOR if human else None,
                "image_shift_px": list(shift),
                "base_offset_mm": list(BASE_OFFSET_MM),
                "target_px": [round(float(v), 2) for v in target],
                "stage_xy_mm": [round(float(v), 4) for v in stage_xy],
                "surface_z_mm": round(surface_z, 4),
                "slide": benches.measurement,
                "detector_restarts": list(getattr(detector, "restarts", [])),
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
        if hasattr(detector, "close"):  # the detector's own process
            detector.close()
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
