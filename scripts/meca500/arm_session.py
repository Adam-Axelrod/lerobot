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
import threading
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
            title=f"{self.title}  [{self.phase}]  q = abort\n{dt}   |   x detected tip   + target   o recorded touches",
            points=[*self.marks, *self.overlays.values()],
        )
        return self.last_frame

    def sleep(self, seconds: float) -> None:
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self.update()


def wait_idle(meca, display: Display, timeout_s: float = 60.0) -> None:
    """WaitIdle, keeping the display (and the abort key) live.

    ONE WaitIdle runs in a background thread while this thread refreshes the display. Calling
    WaitIdle repeatedly with a short timeout does not work: each call queues a NEW checkpoint and
    waits for the controller's round trip, so with a 50 ms timeout it never completed - the first
    auto_calibrate run sat "moving home" with the arm already at home until the 60 s timeout.
    """
    box: dict = {}

    def _wait() -> None:
        try:
            meca.WaitIdle(timeout=timeout_s)
        except Exception as e:  # handed back to the caller's thread below
            box["error"] = e

    worker = threading.Thread(target=_wait, daemon=True)
    worker.start()
    while worker.is_alive():
        display.update()  # raises AbortedError on q; the caller then ClearMotion()s the arm
    if "error" in box:
        raise TimeoutError(f"Arm did not become idle: {box['error']}") from box["error"]


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


def guarded_descent(
    meca, kin, detector, display: Display, floor_z: float, speed_mm_s: float, vision=None
) -> dict:
    """Creep straight down until contact or until the tip reaches ``floor_z`` (an over-travel
    limit). Pure translation, so the tool's gravity torque is constant and any change is
    contact. Leaves the arm stopped; returns {contact, reason, d_torque_mnm, d_force_n, tip_z, q}
    plus, with ``vision``, ``px`` and ``q_touch``: the pixel and joints at touch-down.

    ``vision`` (optional) has reset(), check(frame, q) -> (contact, px_or_None) and a
    ``touchdown`` (px, q) of the last frame before the tip began to slide. On the arm the
    shallow pipette slides forward along the dish the moment it touches, long before the torque
    crosses its threshold (a millimetre or more on 2026-10-14) - the camera sees that slide first."""
    display.sleep(0.5)  # let vibrations die before the baseline
    detector.capture_baseline(0.5)
    if vision is not None:
        vision.reset()
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
            if vision is not None:
                hit, px = vision.check(display.last_frame, q)
                if hit:
                    result = {
                        "contact": True,
                        "reason": "vision: tip started sliding",
                        "d_torque_mnm": reading.d_torque_mnm,
                        "d_force_n": reading.d_force_n,
                        "px": px,
                    }
                    break
    finally:
        meca.MoveLinVelWrf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    wait_idle(meca, display, 5)
    q = np.array(meca.GetRtJointPos())
    result["q"] = q
    result["tip_z"] = float(kin.tip(q)[2, 3])
    # The arm only stops after contact is sensed, by which time the tip has slid along the dish
    # and the flange has gone on down. Where it TOUCHED is the last frame before the slide.
    if result.get("contact") and vision is not None and vision.touchdown is not None:
        result["px"], result["q_touch"] = vision.touchdown
        result["tip_z"] = float(kin.tip(result["q_touch"])[2, 3])
    return result
