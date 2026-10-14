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

Reports each contact's height against the scope calibration's dish surface, and the spread
between trials - the repeatability the automatic calibration can rely on.

Uses the rig's Bota sensor (rigs.py BOTA_SENSORS) and the bench model in assets/benches/<RIG>/.
Shows the microscope feed with a contact banner (green NO CONTACT / red CONTACT) and the live
torque change against the threshold. Owns the arm and the camera: close teleop and the web portal
first. Press q in the window (or close it) to stop the arm immediately.
"""

import contextlib
import logging
import sys
import time
from dataclasses import replace

import numpy as np
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
DESCENT_MM_S = 0.15  # creep speed; the stop lag (~20 ms) is then ~3 um of over-travel
MAX_OVERTRAVEL_MM = 2.0  # abort if this far below the calibrated dish without a contact
CONTACT_TORQUE_MNM = 0.3  # ~3-6x the smoothed torque noise measured by bota_probe.py
FORCE_LIMIT_N = 0.05  # hard backstop (user-agreed)
TRANSIT_SPEED_DEG_S = 10.0
LIFT_SPEED_MM_S = 2.0
# -----------------------------------------------------------------


class AbortedError(Exception):
    """The user pressed q / closed the window."""


class Display:
    """Microscope feed with a contact banner: green NO CONTACT / red CONTACT, the live torque
    change as a bar against the threshold, the phase, and dT / dF. Before the first baseline the
    change is shown against the sensor's tare."""

    def __init__(self, view, robot, detector, camera_key: str):
        self.view, self.robot, self.detector, self.key = view, robot, detector, camera_key
        self.phase = "starting"

    def update(self) -> None:
        import cv2

        from lerobot.robots.meca500.autocollect.contact import evaluate

        if self.view.closed:
            raise AbortedError
        k = self.view.key()
        if k == "q":
            raise AbortedError
        frame = self.robot.get_observation()[self.key].copy()
        m = self.detector.stream.mean(self.detector.th.smooth_s)
        base = self.detector.baseline if self.detector.baseline is not None else np.zeros(6)
        r = evaluate(m, base, self.detector.th) if m is not None else None
        h, w = frame.shape[:2]
        contact = r is not None and r.contact
        colour = (220, 30, 30) if contact else (40, 170, 60)
        cv2.rectangle(frame, (0, 0), (w, 46), colour, -1)
        label = f"CONTACT ({r.reason})" if contact else ("NO CONTACT" if r is not None else "NO SENSOR DATA")
        cv2.putText(frame, label, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        if r is not None:  # torque bar: full width = 2x the contact threshold, tick at the threshold
            frac = min(r.d_torque_mnm / (2 * self.detector.th.torque_mnm), 1.0)
            cv2.rectangle(frame, (0, 46), (int(w * frac), 54), (255, 220, 0), -1)
            cv2.line(frame, (w // 2, 44), (w // 2, 56), (255, 255, 255), 2)
        dt_txt = (
            f"dT {r.d_torque_mnm:.2f} mNm (thr {self.detector.th.torque_mnm})  dF {r.d_force_n:.3f} N"
            if r
            else ""
        )
        self.view.show(frame, title=f"auto_touch_test  [{self.phase}]  q = abort\n{dt_txt}")

    def sleep(self, seconds: float) -> None:
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self.update()


def wait_idle(meca, display: Display, timeout_s: float = 60.0) -> None:
    """WaitIdle, keeping the display (and the abort key) live."""
    end = time.perf_counter() + timeout_s
    while True:
        try:
            meca.WaitIdle(timeout=0.05)
            return
        except Exception:
            if time.perf_counter() > end:
                raise TimeoutError("Arm did not become idle in time.") from None
            display.update()


def run_joint_path(meca, qs, display: Display, timeout_s: float = 60.0) -> None:
    """Queue a collision-checked joint path (one MoveJoints per sample) and wait for it."""
    for q in qs:
        meca.MoveJoints(*[float(v) for v in q])
    wait_idle(meca, display, timeout_s)


def make_ready(meca) -> None:
    status = meca.GetStatusRobot()
    if status.error_status:
        meca.ResetError()
        meca.WaitErrorReset(timeout=10)
    meca.ClearMotion()
    meca.ResumeMotion()
    with contextlib.suppress(Exception):
        meca.WaitMotionResumed(timeout=5)
    if meca.GetStatusRobot().pause_motion_status:
        raise RuntimeError("Arm still paused after ResumeMotion - check the e-stop and the web portal.")


def guarded_descent(meca, kin, detector, surface_z: float, display: Display) -> dict:
    """Creep straight down until contact; return what happened. Leaves the arm stopped."""
    detector.capture_baseline(0.5)
    display.phase = "descending"
    t0 = time.perf_counter()
    try:
        while True:
            meca.MoveLinVelWrf(0.0, 0.0, -DESCENT_MM_S, 0.0, 0.0, 0.0)
            reading = detector.check()
            q = np.array(meca.GetRtTargetJointPos(synchronous_update=False))
            tip_z = kin.tip(q)[2, 3]
            if reading.contact:
                result = {
                    "contact": True,
                    "reason": reading.reason,
                    "d_torque_mnm": reading.d_torque_mnm,
                    "d_force_n": reading.d_force_n,
                }
                break
            if tip_z < surface_z - MAX_OVERTRAVEL_MM:
                result = {
                    "contact": False,
                    "reason": f"over-travel: {MAX_OVERTRAVEL_MM} mm below the dish, no contact",
                }
                break
            display.update()  # also the loop's pacing (~10-20 Hz) and the q-to-abort check
    finally:
        meca.MoveLinVelWrf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    wait_idle(meca, display, 5)
    q = np.array(meca.GetRtJointPos())
    result["tip_z"] = float(kin.tip(q)[2, 3])
    result["seconds"] = time.perf_counter() - t0
    return result


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

    # Hover pose: field-of-view centre, HOVER_MM above the calibrated dish, approach orientation
    # of the measured touch pose.
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
        f"Hover pose over the FOV centre {np.round(xy, 2).tolist()}, {HOVER_MM} mm above the dish "
        f"(calibrated z {calib.surface_z_mm:.3f}); clearance {clr.min_mm:.1f} mm beyond margin."
    )

    th = ContactThresholds(torque_mnm=CONTACT_TORQUE_MNM, force_limit_n=FORCE_LIMIT_N)
    # Window first: if the display cannot open, nothing has connected to the arm yet.
    view = LiveView("auto_touch_test")
    sensor = bota_driver.BotaDriver(str(bota_config_path(RIG)))
    if not (sensor.configure() and sensor.tare() and sensor.activate()):
        sys.exit("Bota sensor failed to configure/tare/activate - check its COM port in the config.")
    stream = WrenchStream(sensor)
    detector = ContactDetector(stream, th)

    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=False))
    robot.connect()
    robot.configure()
    meca = robot.robot
    display = Display(view, robot, detector, bench.camera_key)
    results = []
    try:
        make_ready(meca)
        meca.SetCartLinVel(LIFT_SPEED_MM_S)
        # The descent loop also refreshes the display (~10-20 Hz), so allow more than the default
        # 50 ms between velocity commands before the arm stops on its own.
        meca.SetVelTimeout(0.3)
        q_now = np.array(meca.GetRtJointPos())
        tip_h = kin.tip(q_now)[2, 3] - calib.surface_z_mm
        print("Moving home, then to the hover pose ...")
        display.phase = "moving home"
        run_joint_path(meca, planner.plan_transit(q_now, home, retreat=tip_h < 40.0), display)
        display.phase = "moving to hover"
        run_joint_path(meca, planner.plan_transit(home, q_hover, retreat=False), display)

        for i in range(N_TRIALS):
            display.phase = f"trial {i + 1}/{N_TRIALS}: settling"
            display.sleep(0.5)  # let vibrations die before the baseline
            r = guarded_descent(meca, kin, detector, calib.surface_z_mm, display)
            results.append(r)
            if r["contact"]:
                print(
                    f"  trial {i + 1}: CONTACT ({r['reason']}) at tip z {r['tip_z']:.3f} mm = "
                    f"{r['tip_z'] - calib.surface_z_mm:+.3f} vs calibrated dish; dT {r['d_torque_mnm']:.2f} mNm, "
                    f"dF {r['d_force_n']:.3f} N, {r['seconds']:.1f} s"
                )
            else:
                print(f"  trial {i + 1}: STOPPED - {r['reason']} (tip z {r['tip_z']:.3f})")
            # back up to the hover height, straight up
            lift = calib.surface_z_mm + HOVER_MM - r["tip_z"]
            display.phase = f"trial {i + 1}/{N_TRIALS}: lifting"
            meca.MoveLinRelWrf(0.0, 0.0, float(lift), 0.0, 0.0, 0.0)
            wait_idle(meca, display, 30)
            if not r["contact"]:
                break

        print("Returning home ...")
        display.phase = "returning home"
        q_now = np.array(meca.GetRtJointPos())
        try:
            run_joint_path(meca, planner.plan_transit(q_now, home, retreat=True), display)
        except PlanningError as e:
            print(f"Could not plan the way home ({e}); the arm holds its pose.")
    except AbortedError:
        print("AbortedError by the user - stopping the arm where it is (it holds its pose).")
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
