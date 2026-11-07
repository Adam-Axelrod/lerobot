"""No-motion check of Huanyu's tip detector (ActionStateRuntime) on the live microscope feed.

    python scripts/meca500/asr_live_check.py

Connects to the robot read-only (monitor mode: nothing can move) and runs the detector on N_FRAMES
live frames, printing each detection, where forward kinematics and the scope calibration put the
tip, and the time per frame. Run it before the first policy_asr session:
    - arm at home (tip out of view): expect no detection at or above eval_auto.WATCH_MIN_CONFIDENCE;
    - optionally, tip jogged into view by hand beforehand: expect the detection within a few tens of
      px of the calibrated tip (they differ by the calibration's error, which the servo does not need).
The first frame is slow while the ONNX model loads. Frames go to outputs/asr_live_check/.
"""

import os
import sys
import tempfile
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
from rigs import rig_config

from lerobot.robots.meca500.autocollect.asr_bridge import AsrTipDetector
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
ASR_SITE_YAML = "assets/benches/meca500_basler/asr_site.yaml"
N_FRAMES = 20
# -----------------------------------------------------------------


def main() -> None:
    import cv2
    from action_state_runtime.config.runtime import load_config
    from action_state_runtime.runtime import ActionStateRuntime

    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    kin, calib = bench.kinematics(), bench.calibration()
    target = bench.section("target")
    runtime = ActionStateRuntime(config=load_config(ASR_SITE_YAML), backend_mode="real", audit_dir=tempfile.mkdtemp())
    detector = AsrTipDetector(runtime, target["field_centre_px"], target["field_radius_px"])
    out = Path("outputs/asr_live_check") / time.strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True)

    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=True))
    robot.connect()
    try:
        for i in range(N_FRAMES):
            obs = robot.get_observation()
            frame = obs[bench.camera_key]
            q = np.array([obs[f"joint_{j + 1}.pos"] for j in range(6)], dtype=float)
            tip = kin.tip(q)[:3, 3]
            fk_px = calib.xy_to_pixel(tip[:2])[0]
            t = time.perf_counter()
            s = detector.detect(frame)
            dt = time.perf_counter() - t
            cv2.imwrite(str(out / f"frame{i:02d}.png"), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            seen = "nothing" if s is None else f"tip at ({s.px[0]:.0f}, {s.px[1]:.0f}) confidence {s.confidence:.2f}"
            print(
                f"frame {i:2d}: {seen} | calibrated tip ({fk_px[0]:.0f}, {fk_px[1]:.0f}), "
                f"{(tip[2] - calib.surface_z_mm):.2f} mm above the dish | {dt:.2f} s"
            )
    finally:
        robot.disconnect()
    print(f"frames -> {out}")
    sys.stdout.flush()
    os._exit(0)  # the camera and ONNX threads can hold the interpreter open on Windows


if __name__ == "__main__":
    main()
