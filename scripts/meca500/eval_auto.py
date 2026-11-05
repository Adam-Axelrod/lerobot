"""Evaluate a policy trained on record_auto data, on the bench, under the same conditions.

Each episode is set up exactly as record_auto set up a demonstration - same start-pose box,
stage positions and target region, from SEED_BASE seeds the training set never used - and then
the POLICY drives the arm instead of the scripted expert:

    1. plan  - stage position, start pose and target pixel (the scripted path is discarded);
    2. transit (collision-checked, scripted) to the start pose, then the stage moves;
    3. rollout - the marker appears and the policy runs at the recording rate. Every command is
               checked BEFORE it is sent: per-step clamp, collision clearance of the commanded
               pose against the bench model, and the tip height above the dish height map
               (no pressing into the dish). A failed check ends the episode where it stands;
    4. measure - final tip position from forward kinematics AND from the tip detector on the
               raw frame, both against the target, in px and um; raw frames are saved for the
               by-eye check, plus the joint trajectory for the smoothness metric.

The robot runs with relative_actions=False throughout: this script integrates the policy's
delta onto the controller's current target itself, so that it can check the resulting pose
before sending it. Transits are absolute scripted moves, as in record_auto.

Results go to outputs/eval_auto/<EVAL_NAME>_<timestamp>/ - results.jsonl (one line per
episode), epNNN/{background,start,final,final_overlay}.png and epNNN/traj.npz.
Keys: right arrow ends an episode early (it is still measured), Esc stops the session.
"""

import contextlib
import json
import logging
import sys
import time
from pathlib import Path

import numpy as np
import torch
from record_auto import (
    SETTLE_TIMEOUT_S,
    StagePlanner,
    build_planner,
    connect_stage,
    go_home,
    grab,
    make_ready,
    run_plan,
    save_raw,
    settled_background,
    wait_settled,
)
from rigs import require_home_joints, rig_config

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
from lerobot.robots.meca500.autocollect.focus_check import FocusMismatchError, check_focus
from lerobot.robots.meca500.autocollect.overlay import TargetOverlayStep, draw_target
from lerobot.robots.meca500.autocollect.planner import PlanningError
from lerobot.robots.meca500.autocollect.tip_detect import detect_tip
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted
from lerobot.utils.constants import OBS_STR
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.keyboard_input import init_keyboard_listener, is_headless
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import init_logging, log_say

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
# A local pretrained_model folder or a Hub model repo (downloaded once into the HF cache).
# Must be trained on a DELTA dataset recorded on this rig with the digital target.
# Local copies come from CREATE by scp: the lab PC downloads from the Hub at ~100 KB/s.
MODEL = "outputs/train/basler_stage_touch_2026-11_delta/checkpoints/100000/pretrained_model"
EVAL_NAME = "basler_stage_touch_2026-11_delta_act"  # results go to outputs/eval_auto/<EVAL_NAME>_<time>/
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
# prediction ~5% weight and a mean prediction age of ~0.4 s. Compare against None if the
# approach looks sluggish or oscillates.
TEMPORAL_ENSEMBLE_COEFF: float | None = -0.05

# Per-step clamps (deg), just above the largest step in the 200 training episodes
# (coarse max 0.956, precision max 0.0167; p99 0.61 and 0.014): they never touch
# demonstrated motion and stop a runaway.
MAX_REL_TARGET = 1.0
PRECISION_MAX_REL_TARGET = 0.02
# The commanded tip may go this far below the dish height map before the episode is stopped.
# The demos touched AT the surface (touch_offset_mm 0); the map drifts by up to ~40 um a day.
PRESS_LIMIT_MM = 0.1
# The episode ends once the policy has been in precision mode and the commanded tip has moved
# less than SETTLED_UM_PER_STEP per step for SETTLED_S (the demos end with a 0.5-1.5 s pause).
SETTLED_S = 1.0
SETTLED_UM_PER_STEP = 1.0
SUCCESS_UM = 50.0
DEVICE = "cuda"
# -----------------------------------------------------------------


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
    pre, post = make_pre_post_processors(
        cfg, pretrained_path=str(path), preprocessor_overrides={"device_processor": {"device": DEVICE}}
    )
    logging.info(f"Policy {cfg.type} from {path} (trained on {dataset})")
    return policy, pre, post


def rollout_episode(robot, policy, pre, post, obs_proc, obs_features, planner, events) -> dict:
    """Run the policy until it settles, the time runs out, a check fails or the operator stops it."""
    kin, checker, calib = planner.kin, planner.checker, planner.calib
    fps = planner.lim.fps
    policy.reset()
    pre.reset()
    post.reset()
    log = {k: [] for k in ("t", "q_cmd", "q_meas", "delta", "precision", "clamped")}
    reason, settled, ever_precise = "time limit", 0, False
    prev_tip = None
    t_start = time.perf_counter()
    while time.perf_counter() - t_start < EPISODE_TIME_S:
        t0 = time.perf_counter()
        if events["exit_early"] or events["stop_recording"]:
            reason = "operator"
            break
        obs = obs_proc(robot.get_observation())
        frame = build_dataset_frame(obs_features, obs, prefix=OBS_STR)
        action = predict_action(frame, policy, torch.device(DEVICE), pre, post, False, TASK, robot.name)
        a = action.squeeze(0).float().cpu().numpy()
        precise = bool(a[6] > 0.5)
        ever_precise |= precise
        clamp = PRECISION_MAX_REL_TARGET if precise else MAX_REL_TARGET
        delta = np.clip(a[:6], -clamp, clamp)
        q_cmd = np.asarray(robot.robot.GetRtTargetJointPos(), dtype=float) + delta

        tip = kin.tip(q_cmd)[:3, 3]
        height = float(tip[2] - calib.surface_z_mm)
        if height < -PRESS_LIMIT_MM:
            reason = f"stopped: commanded tip {height * 1000:.0f} um below the dish surface"
            break
        clear = checker.clearance(q_cmd)
        if clear.min_mm < 0:
            reason = f"stopped: {clear.where} within margin of {clear.obstacle} ({clear.min_mm:.1f} mm)"
            break

        robot.send_action({**{f"joint_{i + 1}.pos": float(q_cmd[i]) for i in range(6)}, "precision.state": float(precise)})
        log["t"].append(t0 - t_start)
        log["q_cmd"].append(q_cmd)
        log["q_meas"].append(np.asarray(robot.robot.GetRtJointPos(), dtype=float))
        log["delta"].append(a[:6])
        log["precision"].append(precise)
        log["clamped"].append(bool(np.any(np.abs(a[:6]) > clamp)))

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
    return {"reason": reason, "duration_s": round(time.perf_counter() - t_start, 2), "loop_hz": round(hz, 1), "log": log}


def measure_final(robot, planner, bench, background, target_px) -> tuple[dict, np.ndarray]:
    """Tip vs target by forward kinematics and by the tip detector, after the arm stops."""
    kin, calib = planner.kin, planner.calib
    robot.robot.WaitIdle(timeout=10)
    q = np.asarray(robot.robot.GetRtJointPos(), dtype=float)
    tip = kin.tip(q)[:3, 3]
    um_per_px = calib.mm_per_px * 1000
    fk_px = calib.xy_to_pixel(tip[:2])[0]
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


def main() -> None:
    init_logging()
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    try:
        pos = check_focus(bench.section("scope"))
    except FocusMismatchError as e:
        sys.exit(f"\nREFUSING TO START: {e}\n")
    print(f"Focus check passed: K-Cube at {pos:.4f} mm, as calibrated.")
    policy, pre, post = load_policy(MODEL)

    planner = build_planner(bench)
    stage, model, hmap = connect_stage(bench, planner.calib)
    stager = StagePlanner(planner, model, hmap, planner.calib)
    home = np.asarray(require_home_joints(RIG), dtype=float)
    fps = planner.lim.fps

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
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR, use_video=True)

    out_dir = Path("outputs/eval_auto") / f"{EVAL_NAME}_{time.strftime('%Y%m%d-%H%M%S')}"
    out_dir.mkdir(parents=True)
    (out_dir / "config.json").write_text(
        json.dumps(
            {k: v for k, v in globals().items() if k.isupper() and isinstance(v, (str, int, float, type(None)))},
            indent=2,
        ),
        encoding="utf-8",
        newline="\n",
    )
    results = []

    robot.connect()
    robot.configure()
    teleop.connect()
    listener, events = init_keyboard_listener()
    try:
        make_ready(robot.robot)
        if not stage.is_homed:
            if np.abs(np.array(robot.robot.GetRtJointPos()) - home).max() > 0.2:
                sys.exit("The stage is not homed. Home it in Kinesis with the arm at home (or rerun with the arm at home).")
            stage.home()
        stage_xy = stage.position()
        stager.place(stage_xy)
        q_now = go_home(robot, teleop, planner, home, events, processors)
        retreat = False
        for i in range(NUM_EPISODES):
            if events["stop_recording"]:
                break
            try:
                new_xy, plan = stager.plan_episode(SEED_BASE + i)
                stager.place(stage_xy)
                stager.check_parked_arm(plan.start_joints, stage_xy, new_xy)
                transit = planner.plan_transit(q_now, plan.start_joints, retreat=retreat)
            except PlanningError as e:
                logging.error(str(e))
                break
            overlay.set_target(None)
            log_say(f"Moving to start of episode {i}", False)
            run_plan(robot, teleop, transit, np.zeros(len(transit)), fps, events, processors)
            wait_settled(robot, plan.start_joints, SETTLE_TIMEOUT_S)
            stage_xy = stage.move_to(new_xy)
            surface_z = stager.place(stage_xy)
            background, _ = settled_background(robot, bench.camera_key)
            ep_dir = out_dir / f"ep{i:03d}"
            save_raw(ep_dir, "background", background)

            target = np.asarray(plan.target_px, dtype=float)
            overlay.set_target(plan.target_px)
            log_say(f"Policy episode {i}", False)
            run = rollout_episode(robot, policy, pre, post, obs_proc, obs_features, planner, events)
            m, final = measure_final(robot, planner, bench, background, target)
            overlay.set_target(None)
            save_raw(ep_dir, "final", final)
            save_raw(ep_dir, "final_overlay", draw_target(final, target, bench.target_style()))
            np.savez_compressed(ep_dir / "traj.npz", **{k: np.asarray(v) for k, v in run["log"].items()})

            row = {
                "episode": i,
                "seed": SEED_BASE + i,
                "target_px": [round(float(v), 2) for v in target],
                "stage_xy_mm": [round(float(v), 4) for v in stage_xy],
                "surface_z_mm": round(surface_z, 4),
                "reason": run["reason"],
                "duration_s": run["duration_s"],
                "loop_hz": run["loop_hz"],
                "steps": len(run["log"]["t"]),
                "clamped_steps": int(np.sum(run["log"]["clamped"])),
                **m,
            }
            results.append(row)
            with (out_dir / "results.jsonl").open("a", encoding="utf-8", newline="\n") as f:
                f.write(json.dumps(row) + "\n")
            logging.info(
                f"episode {i}: {run['reason']} after {run['duration_s']} s | FK error {m['fk_error_um']:.0f} um, "
                f"vision {m.get('vision_error_um', float('nan')):.0f} um, tip {m['tip_height_um']:+.0f} um"
            )
            if robot.robot.GetStatusRobot().error_status:
                raise RuntimeError("Robot reported an error during the episode; stopping.")
            q_now = np.array(robot.robot.GetRtJointPos())
            retreat = True

        events["exit_early"] = False
        try:
            go_home(robot, teleop, planner, home, events, processors)
        except RuntimeError as e:
            logging.error(f"Could not return home ({e}); the arm holds its current pose.")
    finally:
        overlay.set_target(None)
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
