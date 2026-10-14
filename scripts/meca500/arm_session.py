"""Shared arm + display helpers for the automatic bench scripts (auto_touch_test, auto_calibrate).

- Display: the microscope feed with a contact banner (green NO CONTACT / red CONTACT), a live
  torque bar against the contact threshold, the current phase and message, and markers
  (detected tip, predicted tip, targets, collected points). Pressing q / closing the window
  raises AbortedError from any update.
- wait_idle / run_joint_path: queue motion and wait, keeping the display live.
- make_ready: clear errors, stale queued motion and a resettable safety stop.
- guarded_descent: creep straight down until contact (torque-based, see autocollect/contact.py).

All motion here is either a collision-checked joint path from the planner, or a short straight
move/descent near the dish that the caller has checked.
"""

from __future__ import annotations

import contextlib
import time

import cv2
import numpy as np

from lerobot.robots.meca500.autocollect.contact import evaluate


class AbortedError(Exception):
    """The user pressed q or closed the window."""


class Display:
    def __init__(self, view, robot, detector, camera_key: str, title: str):
        self.view, self.robot, self.detector, self.key, self.title = view, robot, detector, camera_key, title
        self.phase = "starting"
        self.message = ""
        self.marks: list = []  # (u, v, colour[, style]) - persistent markers, e.g. collected points
        self.overlays: dict = {}  # name -> (u, v, colour, style) - transient: detected/predicted/target
        self.last_frame: np.ndarray | None = None

    def frame(self) -> np.ndarray:
        """A fresh camera frame (also kept as last_frame)."""
        self.last_frame = self.robot.get_observation()[self.key]
        return self.last_frame

    def update(self) -> np.ndarray:
        if self.view.closed or self.view.key() == "q":
            raise AbortedError
        frame = self.frame().copy()
        m = self.detector.stream.mean(self.detector.th.smooth_s)
        base = self.detector.baseline if self.detector.baseline is not None else np.zeros(6)
        r = evaluate(m, base, self.detector.th) if m is not None else None
        h, w = frame.shape[:2]
        contact = r is not None and r.contact
        cv2.rectangle(frame, (0, 0), (w, 46), (220, 30, 30) if contact else (40, 170, 60), -1)
        label = f"CONTACT ({r.reason})" if contact else ("NO CONTACT" if r is not None else "NO SENSOR DATA")
        cv2.putText(frame, label, (8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        if r is not None:  # torque bar: full width = 2x the threshold; white tick = the threshold
            frac = min(r.d_torque_mnm / (2 * self.detector.th.torque_mnm), 1.0)
            cv2.rectangle(frame, (0, 46), (int(w * frac), 54), (255, 220, 0), -1)
            cv2.line(frame, (w // 2, 44), (w // 2, 56), (255, 255, 255), 2)
        if self.message:
            cv2.rectangle(frame, (0, h - 28), (w, h), (0, 0, 0), -1)
            cv2.putText(
                frame, self.message[:60], (6, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1
            )
        dt = f"dT {r.d_torque_mnm:.2f} mNm  dF {r.d_force_n:.3f} N" if r is not None else ""
        self.view.show(
            frame,
            title=f"{self.title}  [{self.phase}]  q = abort\n{dt}",
            points=[*self.marks, *self.overlays.values()],
        )
        return self.last_frame

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


def guarded_descent(meca, kin, detector, display: Display, floor_z: float, speed_mm_s: float) -> dict:
    """Creep straight down until contact or until the tip reaches ``floor_z`` (an over-travel
    limit). Pure translation, so the tool's gravity torque is constant and any change is
    contact. Leaves the arm stopped; returns {contact, reason, d_torque_mnm, d_force_n, tip_z, q}."""
    display.sleep(0.5)  # let vibrations die before the baseline
    detector.capture_baseline(0.5)
    result: dict = {}
    try:
        while True:
            meca.MoveLinVelWrf(0.0, 0.0, -speed_mm_s, 0.0, 0.0, 0.0)
            reading = detector.check()
            q = np.array(meca.GetRtTargetJointPos(synchronous_update=False))
            if reading.contact:
                result = {
                    "contact": True,
                    "reason": reading.reason,
                    "d_torque_mnm": reading.d_torque_mnm,
                    "d_force_n": reading.d_force_n,
                }
                break
            if kin.tip(q)[2, 3] < floor_z:
                result = {"contact": False, "reason": "over-travel limit, no contact"}
                break
            display.update()  # also paces the loop and checks the abort key
    finally:
        meca.MoveLinVelWrf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    wait_idle(meca, display, 5)
    q = np.array(meca.GetRtJointPos())
    result["q"] = q
    result["tip_z"] = float(kin.tip(q)[2, 3])
    return result
