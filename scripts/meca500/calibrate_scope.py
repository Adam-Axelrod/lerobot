"""Calibrate microscope pixels <-> robot XY at the dish surface, for one bench.

Jog the pipette tip with the SpaceMouse (Enter toggles fine mode, as in teleop) while the
Basler feed is shown. At each calibration point:

    1. bring the tip down until it TOUCHES the dish and is in focus,
    2. CLICK on the tip in the image  -> records (clicked pixel, tip XY from the arm).

Spread ~12 points over the whole field of view, corners included. Keys (image window focused):

    z   record the dish-surface height (tip touching the dish) - do this at a few places
    f   fit and report residuals; afterwards a cyan cross shows where the fit thinks the
        tip is, live - it should sit on the tip as you jog anywhere in the field
    u   undo the last point
    s   save to assets/benches/<RIG>/scope_calibration.json and quit
    q   quit without saving

The tip position comes from OUR kinematics plus bench.yaml's tool.tip_pose_in_flange -
deliberately the same model the planner uses, so any small tool-offset error is absorbed
by the fit instead of turning into a target offset. Run check_kinematics.py first.

The residual reported here is the label noise of every auto-collected demo (the gap
between the drawn marker and where the tip actually lands), so re-do points rather than
accepting a poor fit. Owns the camera and the arm: close teleop/record first.
"""

import logging
import sys
from dataclasses import replace

import cv2
import numpy as np
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.calibration import ScopeCalibration
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
GAIN_TR, GAIN_ROT = 20.0, 15.0  # coarse jog, mm/s and deg/s - gentler than teleop near the dish
GAIN_TR_FINE, GAIN_ROT_FINE = 1.0, 1.0
DISPLAY_SCALE = 1.4
MIN_POINTS = 6
# -----------------------------------------------------------------

WINDOW = "calibrate_scope  (click = point at tip, z = surface, f = fit, u = undo, s = save, q = quit)"


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()  # raises if tool.tip_pose_in_flange is missing - it is required here
    cam_key = bench.camera_key

    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=True))
    teleop = make_teleoperator_from_config(
        Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
            gain_tr_fine=GAIN_TR_FINE,
            gain_rot_fine=GAIN_ROT_FINE,
            axis_signs=axis_signs(RIG),
            home_joints=require_home_joints(RIG),
        )
    )
    robot.connect()
    teleop.connect()

    pixels, xys, surface_z = [], [], []
    state = {"click": None}
    fit: ScopeCalibration | None = None

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x / DISPLAY_SCALE, y / DISPLAY_SCALE)

    cv2.namedWindow(WINDOW)
    cv2.setMouseCallback(WINDOW, on_mouse)
    saved = False
    try:
        while True:
            teleop.get_action()  # keeps the teleop's state (precision latch) serviced
            obs = robot.get_observation()
            q = np.array([obs[f"joint_{i + 1}.pos"] for i in range(6)])
            tip = kin.tip(q)[:3, 3]
            frame = obs[cam_key]
            h, w = frame.shape[:2]

            if state["click"] is not None:
                pixels.append(state["click"])
                xys.append(tip[:2].copy())
                print(
                    f"  point {len(pixels):2d}: px ({state['click'][0]:.1f}, {state['click'][1]:.1f})  "
                    f"tip xy ({tip[0]:.4f}, {tip[1]:.4f}) z {tip[2]:.4f} mm"
                )
                state["click"] = None

            view = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            view = cv2.resize(view, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE, interpolation=cv2.INTER_LINEAR)
            for u, v in pixels:
                cv2.circle(view, (int(u * DISPLAY_SCALE), int(v * DISPLAY_SCALE)), 4, (0, 0, 255), -1)
            if fit is not None:
                pu, pv = fit.xy_to_pixel(tip[:2])[0] * DISPLAY_SCALE
                cv2.drawMarker(view, (int(pu), int(pv)), (255, 255, 0), cv2.MARKER_CROSS, 24, 1)
            text = f"points {len(pixels)}  surface samples {len(surface_z)}  tip z {tip[2]:.3f} mm"
            cv2.putText(view, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(WINDOW, view)

            key = cv2.waitKey(30) & 0xFF
            if key == ord("z"):
                surface_z.append(tip[2])
                print(f"  surface z sample {len(surface_z)}: {tip[2]:.4f} mm")
            elif key == ord("u") and pixels:
                pixels.pop()
                xys.pop()
                print(f"  undone; {len(pixels)} points")
            elif key in (ord("f"), ord("s")):
                if len(pixels) < MIN_POINTS or not surface_z:
                    print(
                        f"  need >= {MIN_POINTS} points and >= 1 surface sample (have {len(pixels)}, {len(surface_z)})"
                    )
                    continue
                fit = ScopeCalibration.fit(pixels, xys, float(np.mean(surface_z)), (w, h))
                print(
                    f"  fit: {fit.residual_px_rms:.2f} px rms = {fit.residual_um_rms:.1f} um rms, "
                    f"{fit.mm_per_px * 1000:.2f} um/px, surface z {fit.surface_z_mm:.4f} mm "
                    f"(+/- {np.std(surface_z) * 1000:.0f} um over {len(surface_z)} samples)"
                )
                if key == ord("s"):
                    fit.save(bench.calibration_path())
                    print(f"Saved {bench.calibration_path()}")
                    saved = True
                    break
            elif key == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        teleop.disconnect()
        robot.disconnect()
    if not saved:
        sys.exit("Not saved.")


if __name__ == "__main__":
    main()
