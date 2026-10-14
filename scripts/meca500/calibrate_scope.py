"""Calibrate microscope pixels <-> robot XY at the dish surface, for one bench.

Jog the pipette tip with the SpaceMouse (Enter toggles fine mode, as in teleop) while the
Basler feed is shown. At each calibration point:

    1. bring the tip down until it TOUCHES the dish and is in focus,
    2. CLICK on the tip in the image  -> records (clicked pixel, tip XY from the arm).

Spread ~12 points over the whole field of view, corners included. Keys (image window focused):

    z   record the dish-surface height (tip touching the dish) - do this at a few places;
        it also prints the joints, one of which becomes approach.touch_nominal_joints
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

import numpy as np
from live_view import LiveView
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
    # Window first: if the display cannot open, nothing has connected to the arm yet.
    view = LiveView(WINDOW)
    robot.connect()
    teleop.connect()

    pixels, xys, surface_z = [], [], []
    fit: ScopeCalibration | None = None
    saved = False
    try:
        while True:
            teleop.get_action()  # keeps the teleop's state (precision latch) serviced
            obs = robot.get_observation()
            q = np.array([obs[f"joint_{i + 1}.pos"] for i in range(6)])
            tip = kin.tip(q)[:3, 3]
            frame = obs[cam_key]
            h, w = frame.shape[:2]

            click = view.click()
            if click is not None:
                pixels.append(click)
                xys.append(tip[:2].copy())
                print(
                    f"  point {len(pixels):2d}: px ({click[0]:.1f}, {click[1]:.1f})  "
                    f"tip xy ({tip[0]:.4f}, {tip[1]:.4f}) z {tip[2]:.4f} mm"
                )

            marks = [(u, v, "red") for u, v in pixels]
            if fit is not None:
                pu, pv = fit.xy_to_pixel(tip[:2])[0]
                marks.append((pu, pv, "cyan", "+"))
            text = f"points {len(pixels)}  surface samples {len(surface_z)}  tip z {tip[2]:.3f} mm"
            view.show(frame, title=f"{WINDOW}\n{text}", points=marks)
            if view.closed:
                break

            key = view.key()
            if key == "z":
                surface_z.append(tip[2])
                print(f"  surface z sample {len(surface_z)}: {tip[2]:.4f} mm")
                print(
                    f"    joints {[round(float(j), 2) for j in q]}  <- usable as approach.touch_nominal_joints"
                )
            elif key == "u" and pixels:
                pixels.pop()
                xys.pop()
                print(f"  undone; {len(pixels)} points")
            elif key in ("f", "s"):
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
                if key == "s":
                    fit.save(bench.calibration_path())
                    print(f"Saved {bench.calibration_path()}")
                    saved = True
                    break
            elif key == "q":
                break
    finally:
        view.close()
        teleop.disconnect()
        robot.disconnect()
    if not saved:
        sys.exit("Not saved.")


if __name__ == "__main__":
    main()
