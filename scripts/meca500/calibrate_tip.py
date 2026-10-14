"""Measure where the pipette tip is relative to the flange - and so the angle the tool is
bolted on at - by pivot calibration. Needs the arm and the Basler; no scope calibration.

The CAD gives the tool's shape but not its angle about the flange axis. Pick any sharp
feature in the microscope image (a scratch, a speck, a dot on the dish) and, at each of
5-8 CLEARLY DIFFERENT arm orientations (tilt and twist the wrist by 15-30 deg between them):

    1. jog the tip (SpaceMouse; Enter toggles fine mode) so it TOUCHES that same feature,
       centred on the on-screen crosshair, in focus,
    2. press c to capture.

Every capture puts the same physical point at the tip, so the flange poses pin down the tip
in the flange frame (least squares: R_i t + p_i = const). Keys (image window focused):

    c   capture        u   undo last        f   fit and report
    s   fit, write tool.measured_tip_in_flange into bench.yaml, quit        q   quit

The report compares the measured tip with the model's tip (angle about the flange axis, radius
from it, distance along it). Saving writes the MEASURED tip, which import_bench_cad.py then uses
instead of the CAD's - re-run this after every pipette change or knock, then re-run
import_bench_cad.py and calibrate_scope.py (the scope calibration is computed with the tip).
"""

import logging
import re
import sys
from dataclasses import replace

import numpy as np
from live_view import LiveView
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.kinematics import Meca500Kinematics, rot_z, rotation_log
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
GAIN_TR, GAIN_ROT = 20.0, 15.0
GAIN_TR_FINE, GAIN_ROT_FINE = 1.0, 2.0
MIN_CAPTURES = 4
MIN_ORIENTATION_SPREAD_DEG = 20.0  # largest pairwise flange rotation needed for a good fit
# -----------------------------------------------------------------

WINDOW = "calibrate_tip  (c = capture, u = undo, f = fit, s = save, q = quit)"


def pivot_fit(flanges):
    """Least-squares tip t (flange frame) and fixed point c (base frame): R_i t + p_i = c."""
    a = np.vstack([np.hstack([f[:3, :3], -np.eye(3)]) for f in flanges])
    b = np.concatenate([-f[:3, 3] for f in flanges])
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    t, c = sol[:3], sol[3:]
    res = np.array([np.linalg.norm(f[:3, :3] @ t + f[:3, 3] - c) for f in flanges])
    return t, c, res


def orientation_spread(flanges) -> float:
    return max(
        np.degrees(np.linalg.norm(rotation_log(a[:3, :3] @ b[:3, :3].T))) for a in flanges for b in flanges
    )


def report(t_meas, bench) -> float:
    """Print the comparison with the CAD tip; return the total flange rotation to apply."""
    tool = bench.section("tool")
    current = float(tool.get("flange_rotation_deg", 0.0))
    t_now = np.asarray(tool["tip_pose_in_flange"][:3], float)  # CAD tip with `current` applied
    t_cad = rot_z(np.radians(-current)) @ t_now  # CAD tip as drawn
    ang = lambda v: np.degrees(np.arctan2(v[1], v[0]))  # noqa: E731
    total = (ang(t_meas) - ang(t_cad) + 180) % 360 - 180
    r_meas, r_cad = np.linalg.norm(t_meas[:2]), np.linalg.norm(t_cad[:2])
    print(f"  measured tip (FRF) {np.round(t_meas, 2).tolist()} mm")
    print(f"  CAD tip      (FRF) {np.round(t_cad, 2).tolist()} mm (as drawn)")
    print(f"  -> rotate the tool {total:+.2f} deg about the flange axis (currently {current:+.2f})")
    print(
        f"  geometry check: radius from axis {r_meas:.2f} vs CAD {r_cad:.2f} mm, "
        f"along axis {t_meas[2]:.2f} vs CAD {t_cad[2]:.2f} mm"
    )
    if abs(r_meas - r_cad) > 2 or abs(t_meas[2] - t_cad[2]) > 2:
        print("  note: the tip differs from the CAD by > 2 mm. Expected after a pipette change or knock")
        print("        (the measured tip is what gets saved and used). For the ORIGINAL pipette it would")
        print("        mean the CAD holder/pipette geometry is wrong.")
    return float(total)


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    kin = Meca500Kinematics()  # flange only: the tip is what is being measured

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
    flanges, saved = [], False
    try:
        while True:
            teleop.get_action()
            obs = robot.get_observation()
            q = np.array([obs[f"joint_{i + 1}.pos"] for i in range(6)])
            view.show(obs[bench.camera_key], title=f"{WINDOW}\ncaptures {len(flanges)}", crosshair=True)
            if view.closed:
                break
            key = view.key()
            if key == "c":
                flanges.append(kin.flange(q))
                spread = orientation_spread(flanges)
                print(
                    f"  capture {len(flanges)}: joints {np.round(q, 2).tolist()}  orientation spread {spread:.1f} deg"
                )
            elif key == "u" and flanges:
                flanges.pop()
                print(f"  undone; {len(flanges)} captures")
            elif key in ("f", "s"):
                if len(flanges) < MIN_CAPTURES:
                    print(f"  need >= {MIN_CAPTURES} captures (have {len(flanges)})")
                    continue
                spread = orientation_spread(flanges)
                t, c, res = pivot_fit(flanges)
                print(
                    f"\nfit over {len(flanges)} captures, orientation spread {spread:.1f} deg: "
                    f"residual {np.sqrt(np.mean(res**2)):.3f} mm rms, {res.max():.3f} max; "
                    f"touched point (BRF) {np.round(c, 2).tolist()}"
                )
                if spread < MIN_ORIENTATION_SPREAD_DEG:
                    print(
                        f"  !! orientations only {spread:.0f} deg apart - tilt the wrist more for a reliable fit"
                    )
                report(t, bench)  # rotation printed for reference; the TIP is what gets saved
                if key == "s":
                    # Save the measured TIP - what changes whenever a pipette is swapped or
                    # bent. The holder's angle on the flange (tool.flange_rotation_deg) was
                    # measured once and does not change with the pipette, so it is left as is;
                    # the angle printed above is for reference.
                    path = bench.root / "bench.yaml"
                    text = path.read_text(encoding="utf-8")
                    value = f"[{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}]"
                    text, n = re.subn(
                        r"(?m)^  measured_tip_in_flange: .*$", f"  measured_tip_in_flange: {value}", text
                    )
                    if n != 1:
                        sys.exit("Could not find tool.measured_tip_in_flange in bench.yaml.")
                    path.write_text(text, encoding="utf-8", newline="\n")
                    print(
                        f"Wrote tool.measured_tip_in_flange = {value} to {path}. Now run import_bench_cad.py."
                    )
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
