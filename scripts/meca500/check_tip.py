"""Quick check: is the pipette tip still where the model thinks it is? READ-ONLY on the bench.

Use after a knock or a suspected bend, before deciding whether to re-run calibrate_tip.py and
calibrate_scope.py. Writes nothing.

Jog the tip (SpaceMouse; Enter toggles fine mode) until it TOUCHES the dish, in focus. The
window shows:

    cyan cross   where the model + scope calibration predict the tip is, live
    title        the predicted tip height above the calibrated dish surface (~0 when touching)

At 3-5 spots spread over the image, touch the dish and CLICK ON THE REAL TIP: each click logs
how far the cross is from the real tip (px and um) and the height error. Keys:

    u   undo the last check        q / close window   finish and print the verdict

Verdict: unchanged if the tip is within TOL_XY_UM in the image and TOL_Z_MM in height
everywhere; otherwise re-run calibrate_tip.py, import_bench_cad.py, then calibrate_scope.py.
"""

import logging
import sys
from dataclasses import replace

import numpy as np
from live_view import LiveView
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
GAIN_TR, GAIN_ROT = 20.0, 15.0
GAIN_TR_FINE, GAIN_ROT_FINE = 1.0, 1.0
# The scope calibration itself is good to ~40 um rms and a light touch to ~0.1 mm in height,
# so allow about twice that before calling the tip moved.
TOL_XY_UM = 80.0
TOL_Z_MM = 0.25
# -----------------------------------------------------------------

WINDOW = "check_tip  (touch the dish, click the REAL tip; u = undo, q = finish)"


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()
    calib = bench.calibration()
    um_per_px = calib.mm_per_px * 1000

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
    checks = []  # (dx_um, dy_um, dz_mm)
    try:
        while not view.closed:
            teleop.get_action()
            obs = robot.get_observation()
            q = np.array([obs[f"joint_{i + 1}.pos"] for i in range(6)])
            tip = kin.tip(q)[:3, 3]
            pred = calib.xy_to_pixel(tip[:2])[0]
            dz = tip[2] - calib.surface_z_mm

            click = view.click()
            if click is not None:
                d_px = np.asarray(click) - pred
                d_um = d_px * um_per_px
                checks.append((*d_um, dz))
                print(
                    f"  check {len(checks)}: real tip is {np.linalg.norm(d_px):.1f} px = "
                    f"{np.linalg.norm(d_um):.0f} um from the cross (dx {d_um[0]:+.0f}, dy {d_um[1]:+.0f} um); "
                    f"model says the tip is {dz:+.3f} mm above the dish"
                )

            marks = [(pred[0], pred[1], "cyan", "+")]
            view.show(
                obs[bench.camera_key],
                title=f"{WINDOW}\nmodel tip height above dish {dz:+.3f} mm   checks {len(checks)}",
                points=marks,
            )
            key = view.key()
            if key == "u" and checks:
                checks.pop()
                print(f"  undone; {len(checks)} checks")
            elif key == "q":
                break
    finally:
        view.close()
        teleop.disconnect()
        robot.disconnect()

    if not checks:
        sys.exit("No checks taken.")
    c = np.array(checks)
    xy = np.linalg.norm(c[:, :2], axis=1)
    print(
        f"\n{len(c)} checks: tip offset in the image {xy.mean():.0f} um mean, {xy.max():.0f} um max "
        f"(mean direction dx {c[:, 0].mean():+.0f}, dy {c[:, 1].mean():+.0f} um); "
        f"height error {c[:, 2].mean():+.3f} mm mean, {np.abs(c[:, 2]).max():.3f} mm max"
    )
    if xy.max() <= TOL_XY_UM and np.abs(c[:, 2]).max() <= TOL_Z_MM:
        print("UNCHANGED - the tip is where the model expects. No recalibration needed.")
    else:
        print(
            "MOVED - the tip is not where the model expects. Re-run calibrate_tip.py, then "
            "import_bench_cad.py (Claude), then calibrate_scope.py."
        )


if __name__ == "__main__":
    main()
