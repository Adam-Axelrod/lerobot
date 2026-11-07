"""Measure where ASR's tip point sits relative to the pipette's free end, at the servo height.

    python scripts/meca500/asr_tip_offset.py

ASR's detector puts "the tip" back along the shaft from the free end (2026-11-07: its servo
converged on its own point and left the free end ~340 um short of the ring). eval_auto aims ASR
at ring + offset, so this measures the offset where it matters: with the tip held SERVO_HEIGHT_MM
above the dish, as during the servo.

For a GRID x GRID grid of pixels GRID_STEP_PX apart around the lit field's centre: plan a normal
episode to that pixel (collision-checked, no stage move), fly it to its pre-touch point, move along
the pipette to SERVO_HEIGHT_MM above the touch point, settle, and read N_FRAMES frames: ASR's tip
px, our free-end detector's px (background-subtracted, background taken at home) and the forward-
kinematics tip projected with the scope calibration. Then back home along a checked transit.

Writes assets/benches/<rig>/asr_tip_offset.json: offset_px = median over all frames of
(ASR - free end), its IQR, the per-point medians and their range across the field (if that range
is over MAX_FIELD_RANGE_PX the offset depends on where in the field the tip is: the file says so
and eval_auto should not be run on it before that is dealt with). Moves the arm: hand on the e-stop.
"""

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import record_auto
from eval_auto import SERVO_HEIGHT_MM, deaf, move_tip_to
from record_auto import (
    SETTLE_TIMEOUT_S,
    StagePlanner,
    build_planner,
    connect_stage,
    go_home,
    grab,
    make_ready,
    run_plan,
    settled_background,
    wait_settled,
)
from rigs import require_home_joints, rig_config

from lerobot.processor import make_default_processors
from lerobot.robots.meca500.autocollect.asr_bridge import AsrTipDetector
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.focus_check import check_focus
from lerobot.robots.meca500.autocollect.planner import PlanningError
from lerobot.robots.meca500.autocollect.slide import split_benches
from lerobot.robots.meca500.autocollect.tip_detect import detect_tip
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
ASR_SITE_YAML = "assets/benches/meca500_basler/asr_site.yaml"
GRID = 3
GRID_STEP_PX = 90.0  # 1.2 mm at 13.3 um/px: points within ~1.7 mm of the field centre
N_FRAMES = 5
SEED_BASE = 7_000_000  # planning seeds only (start pose and path); shares nothing with the test set
MAX_FIELD_RANGE_PX = 3.0
# -----------------------------------------------------------------


def main() -> None:
    import tempfile

    from action_state_runtime.config.runtime import load_config
    from action_state_runtime.runtime import ActionStateRuntime

    init_logging()
    register_third_party_plugins()
    benches = split_benches(BenchConfig.load(RIG))
    bench = benches.method
    print(f"Focus at {check_focus(bench.section('scope')):.4f} mm, as calibrated.")
    if not Path(ASR_SITE_YAML).is_file():
        sys.exit(f"{ASR_SITE_YAML} not found")
    runtime = ActionStateRuntime(config=load_config(ASR_SITE_YAML), backend_mode="real", audit_dir=tempfile.mkdtemp())
    runtime.preload_detector()
    tc = bench.section("target")
    detector = AsrTipDetector(runtime, tc["field_centre_px"], tc["field_radius_px"])

    planner = build_planner(bench)
    stage, model, hmap = connect_stage(bench, planner.calib)
    stager = StagePlanner(planner, model, hmap, planner.calib)
    kin, fps = planner.kin, planner.lim.fps
    home = np.asarray(require_home_joints(RIG), dtype=float)
    robot = make_robot_from_config(
        rig_config(
            RIG,
            monitor_mode=False,
            relative_actions=False,
            max_relative_target=record_auto.MAX_REL_TARGET,
            precision_max_relative_target=record_auto.PRECISION_MAX_REL_TARGET,
        )
    )
    teleop = Meca500Scripted(Meca500ScriptedConfig(id="meca500_scripted"))
    processors = make_default_processors()
    key = bench.camera_key
    centre = np.asarray(tc["field_centre_px"], dtype=float)
    offsets = (np.arange(GRID) - (GRID - 1) / 2) * GRID_STEP_PX
    grid = [centre + [du, dv] for dv in offsets for du in offsets]
    points = []

    robot.connect()
    robot.configure()
    teleop.connect()
    try:
        make_ready(robot.robot)
        stage_xy = stage.position()
        surface = stager.place(stage_xy)
        q_now = go_home(robot, teleop, planner, home, deaf(), processors)
        background, _ = settled_background(robot, key)
        for i, px in enumerate(grid):
            try:
                plan = planner.sample_episode(SEED_BASE + i, target_sampler=lambda rng, px=px: px)
                transit = planner.plan_transit(q_now, plan.start_joints, retreat=True)
            except PlanningError as e:
                print(f"point {i} {px.round(1)}: no plan ({e}); skipped")
                continue
            run_plan(robot, teleop, transit, np.zeros(len(transit)), fps, deaf(), processors)
            wait_settled(robot, plan.start_joints, SETTLE_TIMEOUT_S)
            k = plan.pretouch_index
            run_plan(robot, teleop, plan.joints[: k + 1], np.zeros(k + 1), fps, deaf(), processors)
            q = wait_settled(robot, plan.joints[k], SETTLE_TIMEOUT_S)
            touch = np.asarray(plan.target_xyz, dtype=float)
            axis = kin.tip(q)[:3, 2]  # along the pipette, out of the tip (down)
            hover = touch - axis * (SERVO_HEIGHT_MM / -axis[2])
            q = move_tip_to(robot, teleop, planner, hover, 0.5, "to servo height", fps, processors)
            time.sleep(0.5)
            for _ in range(N_FRAMES):
                frame = grab(robot, key, 3)
                a = detector.detect(frame)
                ours = detect_tip(frame, background)
                fk = planner.calib.xy_to_pixel(kin.tip(np.asarray(robot.robot.GetRtJointPos(), float))[:3, 3][:2])[0]
                points.append(
                    {
                        "point": i,
                        "target_px": [round(float(v), 1) for v in px],
                        "asr_px": None if a is None else [round(v, 2) for v in a.px],
                        "asr_conf": None if a is None else round(a.confidence, 3),
                        "free_end_px": None if ours is None else [round(float(v), 2) for v in ours.px],
                        "free_end_score": None if ours is None else round(float(ours.score), 2),
                        "fk_px": [round(float(v), 2) for v in fk],
                        "height_um": round((kin.tip(q)[2, 3] - surface) * 1000, 1),
                    }
                )
            last = points[-1]
            print(f"point {i} {px.round(1)}: ASR {last['asr_px']} ({last['asr_conf']}), free end {last['free_end_px']}, FK {last['fk_px']}, height {last['height_um']} um")
            q_now = np.asarray(robot.robot.GetRtJointPos(), dtype=float)
        go_home(robot, teleop, planner, home, deaf(), processors)
    finally:
        for dev in (robot, teleop, stage):
            try:
                dev.disconnect()
            except Exception:
                pass

    good = [p for p in points if p["asr_px"] and p["free_end_px"] and p["asr_conf"] >= 0.8 and p["free_end_score"] >= 0.3]
    if not good:
        sys.exit("No frame with both detections: nothing measured.")
    d = np.array([np.subtract(p["asr_px"], p["free_end_px"]) for p in good])
    d_fk = np.array([np.subtract(p["asr_px"], p["fk_px"]) for p in good])
    per_point = {}
    for p, v in zip(good, d, strict=True):
        per_point.setdefault(p["point"], []).append(v)
    med = {k: np.median(v, axis=0) for k, v in per_point.items()}
    m = np.array(list(med.values()))
    field_range = (m.max(axis=0) - m.min(axis=0)).tolist()
    out = {
        "offset_px": [round(float(v), 2) for v in np.median(d, axis=0)],
        "iqr_px": [[round(float(v), 2) for v in np.percentile(d[:, j], [25, 75])] for j in range(2)],
        "asr_minus_fk_px": [round(float(v), 2) for v in np.median(d_fk, axis=0)],
        "n": len(good),
        "points": len(med),
        "per_point_median_px": {str(k): [round(float(x), 2) for x in v] for k, v in med.items()},
        "range_across_field_px": [round(v, 2) for v in field_range],
        "field_dependent": bool(max(field_range) > MAX_FIELD_RANGE_PX),
        "height_mm": SERVO_HEIGHT_MM,
        "measured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "frames": points,
    }
    path = bench.root / "asr_tip_offset.json"
    path.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(
        f"\nASR - free end: {out['offset_px']} px (IQR u {out['iqr_px'][0]}, v {out['iqr_px'][1]}), n {out['n']} frames at "
        f"{out['points']} points; ASR - FK {out['asr_minus_fk_px']} px; range across the field {out['range_across_field_px']} px"
        + ("  ** FIELD-DEPENDENT: do not run eval_auto on this **" if out["field_dependent"] else "")
        + f"\n-> {path}"
    )
    sys.stdout.flush()
    os._exit(0)  # camera / ONNX threads can hold the interpreter open on Windows


if __name__ == "__main__":
    main()
