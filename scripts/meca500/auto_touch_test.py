"""Prove force-sensed contact on the real arm: guarded descents onto the dish.

The automatic calibration needs to stop the pipette the moment it touches the dish. This
tests exactly that, and nothing else:

    1. drive home, then to a hover pose HOVER_MM above the dish at the centre of the field of
       view (both collision-checked, as in record_auto);
    2. capture a force/torque baseline with the arm still, then creep STRAIGHT DOWN (pure
       translation - the tool's gravity torque stays constant) at DESCENT_MM_S;
    3. stop when the smoothed torque moves CONTACT_TORQUE_MNM from the baseline (backstops:
       FORCE_LIMIT_N, and an over-travel limit MAX_OVERTRAVEL_MM below the calibrated dish);
    4. record the tip height at contact, lift back to the hover, repeat N_TRIALS times;
    5. return home. The arm holds its pose on disconnect.

Shows the microscope feed with a contact banner (green NO CONTACT / red CONTACT) and the live
torque change against the threshold. Reports each contact's height against the scope
calibration's dish surface, and the spread between trials.

Uses the rig's Bota sensor (rigs.py BOTA_SENSORS) and the bench model in assets/benches/<RIG>/.
Owns the arm and the camera: close teleop and the web portal first. Press q in the window (or
close it) to stop the arm immediately.
"""

import contextlib
import logging
import sys
from dataclasses import replace

import numpy as np
from arm_session import AbortedError, Display, guarded_descent, make_ready, run_joint_path, wait_idle
from live_view import LiveView
from rigs import bota_config_path, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig, PlannerLimits
from lerobot.robots.meca500.autocollect.contact import ContactDetector, ContactThresholds, WrenchStream
from lerobot.robots.meca500.autocollect.kinematics import make_transform
from lerobot.robots.meca500.autocollect.planner import Planner, PlanningError
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
N_TRIALS = 5
HOVER_MM = 3.0  # start each descent this far above the calibrated dish
DESCENT_MM_S = 0.15  # creep speed
MAX_OVERTRAVEL_MM = 2.0  # abort if this far below the calibrated dish without a contact
CONTACT_TORQUE_MNM = 0.3  # ~3-6x the smoothed torque noise measured by bota_probe.py
FORCE_LIMIT_N = 0.05  # hard backstop (user-agreed)
TRANSIT_SPEED_DEG_S = 10.0
LIFT_SPEED_MM_S = 2.0
# -----------------------------------------------------------------


def main() -> None:
    import bota_driver

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()
    calib = bench.calibration()
    limits = replace(PlannerLimits.from_bench(bench), transit_speed_deg_s=TRANSIT_SPEED_DEG_S)
    planner = Planner(kin, bench.collision_checker(kin), calib, limits)
    home = np.asarray(require_home_joints(RIG), dtype=float)

    w, h = calib.image_size
    xy = calib.pixel_to_xy([[w / 2, h / 2]])[0]
    q_touch = np.asarray(limits.touch_nominal_joints, dtype=float)
    t_hover = make_transform(kin.tip(q_touch)[:3, :3], [xy[0], xy[1], calib.surface_z_mm + HOVER_MM])
    q_hover = kin.ik(t_hover, q_touch)
    if q_hover is None:
        sys.exit("No IK for the hover pose.")
    clr = planner.checker.clearance(q_hover)
    if clr.min_mm < 0:
        sys.exit(f"Hover pose collides in the model ({clr.where} vs {clr.obstacle}).")
    print(
        f"Hover over the FOV centre {np.round(xy, 2).tolist()}, {HOVER_MM} mm above the dish "
        f"(calibrated z {calib.surface_z_mm:.3f}); clearance {clr.min_mm:.1f} mm beyond margin."
    )

    view = LiveView("auto_touch_test")  # window first: nothing connected if it cannot open
    sensor = bota_driver.BotaDriver(str(bota_config_path(RIG)))
    if not (sensor.configure() and sensor.tare() and sensor.activate()):
        view.close()
        sys.exit("Bota sensor failed to configure/tare/activate - check its COM port in the config.")
    stream = WrenchStream(sensor)
    detector = ContactDetector(
        stream, ContactThresholds(torque_mnm=CONTACT_TORQUE_MNM, force_limit_n=FORCE_LIMIT_N)
    )

    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=False))
    robot.connect()
    robot.configure()
    meca = robot.robot
    display = Display(view, robot, detector, bench.camera_key, "auto_touch_test")
    results = []
    try:
        make_ready(meca)
        meca.SetCartLinVel(LIFT_SPEED_MM_S)
        meca.SetVelTimeout(0.3)  # the descent loop also refreshes the display (~10-20 Hz)
        q_now = np.array(meca.GetRtJointPos())
        display.phase = "moving home"
        run_joint_path(
            meca,
            planner.plan_transit(q_now, home, retreat=kin.tip(q_now)[2, 3] - calib.surface_z_mm < 40),
            display,
        )
        display.phase = "moving to hover"
        run_joint_path(meca, planner.plan_transit(home, q_hover, retreat=False), display)

        for i in range(N_TRIALS):
            display.phase = f"trial {i + 1}/{N_TRIALS}: descending"
            r = guarded_descent(
                meca, kin, detector, display, calib.surface_z_mm - MAX_OVERTRAVEL_MM, DESCENT_MM_S
            )
            results.append(r)
            if r["contact"]:
                print(
                    f"  trial {i + 1}: CONTACT ({r['reason']}) at tip z {r['tip_z']:.3f} mm = "
                    f"{r['tip_z'] - calib.surface_z_mm:+.3f} vs calibrated dish; dT {r['d_torque_mnm']:.2f} mNm, "
                    f"dF {r['d_force_n']:.3f} N"
                )
            else:
                print(f"  trial {i + 1}: STOPPED - {r['reason']} (tip z {r['tip_z']:.3f})")
            display.phase = f"trial {i + 1}/{N_TRIALS}: lifting"
            meca.MoveLinRelWrf(0.0, 0.0, float(calib.surface_z_mm + HOVER_MM - r["tip_z"]), 0.0, 0.0, 0.0)
            wait_idle(meca, display, 30)
            if not r["contact"]:
                break

        display.phase = "returning home"
        q_now = np.array(meca.GetRtJointPos())
        try:
            run_joint_path(meca, planner.plan_transit(q_now, home, retreat=True), display)
        except PlanningError as e:
            print(f"Could not plan the way home ({e}); the arm holds its pose.")
    except AbortedError:
        print("Aborted by the user - stopping the arm where it is (it holds its pose).")
        with contextlib.suppress(Exception):
            meca.ClearMotion()
    finally:
        with contextlib.suppress(Exception):
            meca.MoveLinVelWrf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        view.close()
        stream.stop()
        with contextlib.suppress(Exception):
            sensor.deactivate()
            sensor.shutdown()
        robot.disconnect()

    z = np.array([r["tip_z"] for r in results if r["contact"]])
    if len(z):
        print(
            f"\n{len(z)}/{len(results)} contacts. Contact height vs calibrated dish: mean "
            f"{(z - calib.surface_z_mm).mean():+.3f} mm, spread (std) {z.std() * 1000:.0f} um, "
            f"range {np.ptp(z) * 1000:.0f} um."
        )


if __name__ == "__main__":
    main()
