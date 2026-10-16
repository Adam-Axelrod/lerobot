"""Automatic tip + scope calibration after a pipette change or knock - no jogging, no clicking.

Runs on the arm with the Basler and the Bota sensor, showing everything on the microscope feed:

    1. HOME + BACKGROUND  drive home (collision-checked); grab a background frame with the tip
                          out of view - tip detection subtracts it.
    2. FIND THE TIP       hover START_ABOVE_DISH_MM over the field-of-view centre, trusting the
                          last calibration (raise it after a pipette change); detect the tip,
                          spiral-search if it is not visible; visually servo it to the centre;
                          descend in view until the tip starts to slide along the dish.
    3. TIP CALIBRATION    at several collision-checked wrist orientations, servo the tip back
                          over the same image point and touch; pivot-fit the tip in the flange
                          frame (corrected for each touch's residual pixel error); save it as
                          tool.measured_tip_in_flange and re-import the tool model.
    4. SCOPE CALIBRATION  touch a grid of points over the visible field; detect the tip at each;
                          fit pixels <-> robot XY; save scope_calibration.json (old one backed up).
    5. VERIFY             touch a few fresh points; report detected-vs-predicted error; go home.

Servoing uses only the old calibration's scale/rotation (camera geometry), which a pipette change
does not affect - only the offset moves, and that is what this re-measures.

On screen: magenta x = detected tip, cyan + = where the model predicts it, yellow + = current
target, green circles = recorded touches, and the force-backstop banner + torque bar. q aborts
(the arm stops and holds its pose).

Contact is sensed by VISION (the tip sliding, see SlideDetector). The Bota sensor is only a
backstop with high thresholds: for the blind fallback (tip not visible), or if the slide is
missed. At the old 0.3 mNm trigger, drift during a long descent fired it in mid-air (2026-10-16).

Owns the arm, the camera and the sensor: close teleop and the web portal first.
"""

import contextlib
import logging
import re
import shutil
import sys
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
from arm_session import AbortedError, Display, guarded_descent, make_ready, run_joint_path, wait_idle
from live_view import LiveView
from rigs import bota_config_path, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig, PlannerLimits
from lerobot.robots.meca500.autocollect.calibration import ScopeCalibration
from lerobot.robots.meca500.autocollect.contact import ContactDetector, ContactThresholds, WrenchStream
from lerobot.robots.meca500.autocollect.kinematics import axis_angle, make_transform
from lerobot.robots.meca500.autocollect.planner import Planner, PlanningError
from lerobot.robots.meca500.autocollect.tip_detect import detect_tip, field_mask
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
# First approach height above the dish of the LAST calibration, with the last measured tip.
# 2 mm is enough when the pipette has not changed. After a pipette change or a knock, the new tip
# can sit several mm lower than the model thinks; set this to ~6 for that run, or the move to the
# start (not guarded) could drive the tip into the dish.
START_ABOVE_DISH_MM = 2.0
HOVER_MM = 1.0  # working hover above the measured dish, once found
FIND_HOVER_MM = 0.5  # search height just above the dish, where the tip is nearly in focus
DESCENT_MM_S = 0.15
FIRST_DESCENT_MM_S = 0.3  # the first, longer descent from the search height
MAX_OVERTRAVEL_MM = 2.5  # never go further than this below the expected dish without contact
SEARCH_STEP_MM = 2.0  # spiral search step if the tip is not in view (field ~5 mm across)
# Centring only needs the tip NEAR the target: what matters is measured at contact, and the
# pivot fit corrects each touch's leftover pixel offset. 2 px stalled on the arm (2026-10-14).
SERVO_TOL_PX = 10.0
PIVOT_TILTS_DEG = 10.0  # tilt about world x and y, both ways
PIVOT_TWIST_DEG = 12.0  # twist about the pipette axis, both ways
MIN_PIVOT_TOUCHES = 4
GRID_OUTER, GRID_INNER = 8, 4  # scope points on two rings inside the visible circle
VERIFY_POINTS = 3
# Force-sensor BACKSTOP only (vision senses contact). Well above the 0.3 mNm / 0.05 N that a
# light touch shows, so sensor drift and noise during a descent cannot trip it.
BACKSTOP_TORQUE_MNM = 1.5
BACKSTOP_FORCE_N = 0.2
TRANSIT_SPEED_DEG_S = 10.0
# The arm runs queued MoveJoints at its global joint velocity, not at the planner's timing, so
# this is what actually sets the speed of every move (the first arm run used the 25 deg/s
# default and the move from home was uncomfortably fast).
JOINT_VEL_DEG_S = 8.0
# Contact by vision: the tip sliding forward along the dish (see SlideDetector).
SLIDE_THRESH_PX = 6.0  # forward travel past the hinge before calling contact (~0.08 mm)
SLIDE_CONFIRM = 3  # frames past the hinge
SLIDE_MIN_RATE = 0.5  # fraction of the geometric slide rate a real slide must reach
SLIDE_HISTORY = 60  # frames kept for the hinge fit
# Only confident (in- or near-focus) detections count: far above the dish the blurred tip's
# detected end wanders by more than the threshold (score ~0.06).
SLIDE_MIN_SCORE = 0.3
MAX_SERVO_STEP_MM = 1.0  # cap each visual-servo move; a blocked step is retried at half size
DEBUG_DIR_ROOT = "outputs/auto_calibrate"  # background + every detection, per run
CART_SPEED_MM_S = 2.0
# -----------------------------------------------------------------

TITLE = "auto_calibrate"


class CalibrationFailedError(RuntimeError):
    pass


class SlideDetector:
    """Contact = the tip starting to slide FORWARD along the dish, seen in the image.

    Descending straight down, the tip's image position barely moves; coming into focus shifts its
    detected end a little (the blurred end fades below the threshold), at a few px per mm. Once it
    touches, the shallow pipette slides forward along the dish at the geometric rate
    1 / (tan(elevation) * mm_per_px) - about 100 px per mm of descent here. So the tip's position
    along the pipette, against the descent, is a HINGE: flat, then rising at that rate. Fit the
    hinge to the recent frames; contact once the rising part is fast enough and has travelled
    SLIDE_THRESH_PX. The touch-down is the hinge point itself - pixel and joints - not the last
    frame, which is the tip after sliding and the flange after going on down.
    """

    def __init__(self, background, kin, lin):
        self.background, self.kin = background, kin
        self.mm_per_px = float(np.sqrt(abs(np.linalg.det(lin))))
        self.reset()

    def reset(self) -> None:
        self.hist = []  # (z, s, px, q)
        self.axis = None
        self.k_exp = None
        self.fit = None

    def check(self, frame, q):
        d = detect_tip(frame, self.background)
        if d is None or d.score < SLIDE_MIN_SCORE:
            return False, None
        q = np.asarray(q, float)
        tip = self.kin.tip(q)
        if self.axis is None:
            # Project on a FIXED axis: the per-frame axis wobbles, and a 1 deg wobble moves the
            # projection of a pixel ~350 px from the origin by ~6 px.
            self.axis = np.asarray(d.axis)
            elev = np.arcsin(np.clip(-tip[2, 2], np.sin(np.radians(5)), 1.0))
            self.k_exp = 1.0 / (np.tan(elev) * self.mm_per_px)  # px per mm of descent
        px = np.asarray(d.px)
        self.hist.append((float(tip[2, 3]), float(px @ self.axis), px, q))
        del self.hist[:-SLIDE_HISTORY]
        self.fit = self._hinge()
        if self.fit is not None:
            z_t, k, n_post = self.fit
            rise = k * (z_t - self.hist[-1][0])
            if k >= SLIDE_MIN_RATE * self.k_exp and rise >= SLIDE_THRESH_PX and n_post >= SLIDE_CONFIRM:
                return True, self.touchdown[0]
        return False, None

    def _hinge(self):
        """Best (z_touch, slope px/mm, frames after) of s = a + k * max(z_t - z, 0), or None."""
        if len(self.hist) < 3 + SLIDE_CONFIRM:
            return None
        z = np.array([h[0] for h in self.hist])
        s = np.array([h[1] for h in self.hist])
        best = None
        for j in range(2, len(z) - SLIDE_CONFIRM):  # frames 0..j flat, j+1.. sliding
            a_mat = np.stack([np.ones_like(z), np.maximum(z[j] - z, 0.0)], axis=1)
            coef, *_ = np.linalg.lstsq(a_mat, s, rcond=None)
            sse = float(np.sum((a_mat @ coef - s) ** 2))
            if coef[1] > 0 and (best is None or sse < best[0]):
                best = (sse, j, coef)
        if best is None:
            return None
        _, j, (a, k) = best
        # Refine between frames: where the sliding line meets the flat level.
        post = slice(j + 1, None)
        if len(z[post]) >= 2:
            k2, b2 = np.polyfit(-z[post], s[post], 1)
            if k2 > 0:
                level = float(np.median(s[max(0, j - 4) : j + 1]))
                z_t = -(level - b2) / k2
                if z[j + 1] <= z_t <= z[max(0, j - 1)]:
                    return z_t, float(k2), len(z) - j - 1
        return float(z[j]), float(k), len(z) - j - 1

    @property
    def touchdown(self):
        """(px, q) at touch-down: the hinge if there is a slide, else the newest frame."""
        if not self.hist:
            return None
        if self.fit is None or self.fit[1] < SLIDE_MIN_RATE * self.k_exp:
            return self.hist[-1][2], self.hist[-1][3]
        z_t = self.fit[0]
        z = np.array([h[0] for h in self.hist])
        flat = [h for h in self.hist if h[0] >= z_t]
        px = np.median([h[2] for h in flat[-3:]], axis=0) if flat else self.hist[0][2]
        # joints at z_t: interpolate (a pure vertical translation, so joints vary smoothly)
        qs = np.array([h[3] for h in self.hist])
        order = np.argsort(z)
        q = np.array([np.interp(z_t, z[order], qs[order, i]) for i in range(qs.shape[1])])
        return px, q


class Session:
    """Everything the stages share: arm, kinematics, model, display, detection."""

    def __init__(self, robot, display, bench, detector):
        self.robot, self.meca, self.display, self.detector = robot, robot.robot, display, detector
        self.background = None
        self.debug_dir = None
        self._n_debug = 0
        self.fov_centre = None
        self.fov_radius = None
        self.reload(bench)

    def reload(self, bench) -> None:
        self.bench = bench
        self.kin = bench.kinematics()
        self.calib = bench.calibration()
        self.limits = replace(PlannerLimits.from_bench(bench), transit_speed_deg_s=TRANSIT_SPEED_DEG_S)
        self.planner = Planner(self.kin, bench.collision_checker(self.kin), self.calib, self.limits)
        self.checker = self.planner.checker
        a = np.asarray(self.calib.px_to_xy)
        self.lin = a[:, :2]  # pixels -> mm: camera scale/rotation, unaffected by the pipette

    # ---------------------------------------------------------------- arm
    def q(self) -> np.ndarray:
        return np.array(self.meca.GetRtJointPos())

    def tip(self, q=None) -> np.ndarray:
        return self.kin.tip(self.q() if q is None else q)

    def move_tip(self, xyz, rot, label: str) -> None:
        """Short collision-checked joint move putting the (model) tip at ``xyz`` with ``rot``."""
        q0 = self.q()
        q1 = self.kin.ik(make_transform(rot, xyz), q0)
        if q1 is None:
            raise CalibrationFailedError(f"No IK for {label}.")
        c = self.checker.check_path(np.vstack([q0, q1]))
        if c.min_mm < 0:
            raise CalibrationFailedError(f"{label} would collide ({c.where} vs {c.obstacle}).")
        self.meca.MoveJoints(*[float(v) for v in q1])
        wait_idle(self.meca, self.display, 60)

    def shift(self, dx: float, dy: float, dz: float, label: str) -> None:
        t = self.tip()
        self.move_tip(t[:3, 3] + [dx, dy, dz], t[:3, :3], label)

    def touch(self, floor_z: float, speed: float) -> dict:
        vision = SlideDetector(self.background, self.kin, self.lin) if self.background is not None else None
        r = guarded_descent(self.meca, self.kin, self.detector, self.display, floor_z, speed, vision)
        if not r["contact"]:
            raise CalibrationFailedError(f"No contact before the over-travel limit (tip z {r['tip_z']:.2f}).")
        print(f"      contact ({r['reason']}) at model tip z {r['tip_z']:.3f}")
        return r

    def contact_px(self, r: dict):
        """The touch-down pixel: the slide detector's (pre-slide) pixel, else a fresh detection."""
        if r.get("px") is not None:
            px = np.asarray(r["px"], dtype=float)
            self.display.overlays["detected"] = (px[0], px[1], "magenta", "x")
            return px
        d = self.detect()
        return None if d is None else np.asarray(d.px)

    def contact_q(self, r: dict) -> np.ndarray:
        """Joints at touch-down (before the slide and the stopping distance), else where it stopped."""
        return r["q_touch"] if r.get("q_touch") is not None else r["q"]

    # ---------------------------------------------------------------- vision
    def detect(self, frames: int = 3):
        """Detect the tip in the newest frame (best score of a few), mark it on screen."""
        best = None
        for _ in range(frames):
            self.display.update()
            d = detect_tip(self.display.last_frame, self.background)
            if d is not None and (best is None or d.score > best.score):
                best = d
        if best is None:
            self.display.overlays.pop("detected", None)
        else:
            self.display.overlays["detected"] = (best.px[0], best.px[1], "magenta", "x")
        self.save_debug("detect", best)
        return best

    def save_debug(self, tag: str, det=None) -> None:
        """Write the latest frame (with the detection, if any) to this run's debug folder."""
        import cv2

        if self.debug_dir is None or self.display.last_frame is None or self._n_debug > 2000:
            return
        img = cv2.cvtColor(self.display.last_frame, cv2.COLOR_RGB2BGR)
        if det is not None:
            u, v = int(round(det.px[0])), int(round(det.px[1]))
            cv2.drawMarker(img, (u, v), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 24, 2)
            cv2.putText(
                img, f"score {det.score:.2f}", (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1
            )
        self._n_debug += 1
        cv2.imwrite(str(self.debug_dir / f"{self._n_debug:04d}_{tag}.png"), img)

    def servo(
        self, target_px, label: str, tol_px: float = SERVO_TOL_PX, max_iter: int = 12, best_effort=False
    ):
        """Translate the tip (at its current height) until it sits on ``target_px`` in the image.

        Gain 0.8: a hovering tip is slightly blurred, so its detected position jitters by a pixel
        or two, and a full-gain step would chase that noise."""
        self.display.overlays["target"] = (target_px[0], target_px[1], "yellow", "+")
        for _ in range(max_iter):
            d = self.detect()
            if d is None:
                raise CalibrationFailedError(f"Lost the tip while servoing ({label}).")
            err = np.asarray(target_px) - np.asarray(d.px)
            self.display.message = f"{label}: tip {np.linalg.norm(err):.1f} px from target"
            if np.linalg.norm(err) <= tol_px:
                return d
            dxy = 0.8 * (self.lin @ err)
            n = float(np.linalg.norm(dxy))
            if n > MAX_SERVO_STEP_MM:
                dxy *= MAX_SERVO_STEP_MM / n
            before = self.tip()[:3, 3].copy()
            try:
                try:
                    self.shift(float(dxy[0]), float(dxy[1]), 0.0, label)
                except CalibrationFailedError:
                    self.shift(float(dxy[0]) / 2, float(dxy[1]) / 2, 0.0, f"{label} (half step)")
            except CalibrationFailedError as e:
                # Centring is a convenience, not a requirement: any tip position well inside the
                # image serves as the reference. Stop where it is rather than failing the run
                # (arm run 4: the model put the illuminator within the margin of the centre).
                if best_effort and self._inside_image(d.px):
                    print(
                        f"    {label}: stopped {np.linalg.norm(err):.0f} px from the target ({e}); continuing from here"
                    )
                    return d
                raise
            moved = self.tip()[:3, 3] - before
            # Logged because the first arm runs stalled ~8 px out with identical detections:
            # did the arm not move, or did the camera not update?
            print(
                f"    servo {label}: tip px {np.round(d.px, 1).tolist()} err {np.linalg.norm(err):.1f} px -> "
                f"commanded {np.round(dxy * 1000).astype(int).tolist()} um, arm moved "
                f"{np.round(moved[:2] * 1000).astype(int).tolist()} um"
            )
        if best_effort and d is not None and self._inside_image(d.px):
            return d
        raise CalibrationFailedError(f"Servo did not converge ({label}).")

    def _inside_image(self, px, margin: float = 40.0) -> bool:
        h, w = self.display.last_frame.shape[:2]
        return margin <= px[0] <= w - 1 - margin and margin <= px[1] <= h - 1 - margin


# ---------------------------------------------------------------------- stages


def stage_home_and_background(s: Session, home) -> None:
    s.display.phase = "1/5 home + background"
    s.display.message = "moving home; the tip must leave the microscope's view"
    q = s.q()
    if np.abs(q - home).max() > 0.2:  # already home: nothing to do
        low = s.tip(q)[2, 3] - s.calib.surface_z_mm < 40
        run_joint_path(s.meca, s.planner.plan_transit(q, home, retreat=low), s.display)
    s.display.sleep(1.0)  # let auto-exposure settle
    frames = [s.display.frame().astype(np.float32) for _ in range(5)]
    s.background = np.median(frames, axis=0).astype(np.uint8)
    if s.debug_dir is not None:
        import cv2

        cv2.imwrite(str(s.debug_dir / "0000_background.png"), cv2.cvtColor(s.background, cv2.COLOR_RGB2BGR))
    mask = field_mask(s.background)
    ys, xs = np.nonzero(mask)
    s.fov_centre = np.array([xs.mean(), ys.mean()])
    h_img, w_img = mask.shape
    # The bright field can run off the image (it does on Huanyu's setup), so the usable radius is
    # also limited by the image borders - keep 25 px clear of them for the tip detector.
    border = (
        min(s.fov_centre[0], w_img - 1 - s.fov_centre[0], s.fov_centre[1], h_img - 1 - s.fov_centre[1]) - 25
    )
    s.fov_radius = float(min(np.sqrt(mask.sum() / np.pi), border))
    s.display.message = f"background captured; visible circle r = {s.fov_radius:.0f} px"
    print(
        f"Background captured. Visible field: centre {np.round(s.fov_centre, 1).tolist()}, r {s.fov_radius:.0f} px"
    )


def spiral_search(s: Session, rot, label: str):
    """Look for the tip here, then on rings 1-3 x SEARCH_STEP_MM around here, at this height.
    Returns the detection (the arm stays where it saw the tip) or None (arm back at the start)."""
    base = s.tip()[:3, 3].copy()
    d = s.detect()
    offsets = [
        (SEARCH_STEP_MM * r * np.cos(a), SEARCH_STEP_MM * r * np.sin(a))
        for r in (1, 2, 3)
        for a in np.radians(np.arange(0, 360, {1: 60, 2: 30, 3: 20}[r]))
    ]
    for off in offsets:
        if d is not None:
            return d
        s.display.message = f"{label}: tip not in view - searching ({off[0]:+.1f}, {off[1]:+.1f}) mm"
        try:
            s.move_tip(base + [off[0], off[1], 0.0], rot, "search move")
        except CalibrationFailedError as e:
            print(f"  search offset ({off[0]:+.1f}, {off[1]:+.1f}) skipped: {e}")
            continue
        d = s.detect()
    if d is None:
        s.move_tip(base, rot, "back to the search start")
    return d


def stage_find_tip(s: Session, home) -> dict:
    """Find the tip just above the old dish and descend onto it in view; blind only as a fallback.

    Start START_ABOVE_DISH_MM above the last calibration's dish: the tip is usually where the
    last calibration left it, and starting 6 mm up made the first descent slow (2026-10-16). If
    the tip cannot be seen there, fall back to finding the dish blind (the force backstop) and
    searching just above it, where the tip is nearly in focus.
    """
    s.display.phase = "2/5 find the tip"
    q_touch = np.asarray(s.limits.touch_nominal_joints, dtype=float)
    rot = s.kin.tip(q_touch)[:3, :3]
    centre_xy = s.calib.pixel_to_xy([s.fov_centre])[0]
    z_old = s.calib.surface_z_mm
    q_hover = s.kin.ik(make_transform(rot, [*centre_xy, z_old + START_ABOVE_DISH_MM]), q_touch)
    if q_hover is None:
        raise CalibrationFailedError("No IK for the start pose above the dish.")
    s.display.message = f"moving to {START_ABOVE_DISH_MM} mm above the last calibrated dish"
    run_joint_path(s.meca, s.planner.plan_transit(s.q(), q_hover, retreat=False), s.display)

    s.display.message = f"looking for the tip from {START_ABOVE_DISH_MM} mm up"
    d = spiral_search(s, rot, f"{START_ABOVE_DISH_MM} mm up")
    if d is not None:
        print(
            f"Tip found from {START_ABOVE_DISH_MM} mm up at px {np.round(d.px, 1).tolist()} (score {d.score:.2f})."
        )
        s.servo(s.fov_centre, "centring", best_effort=True)
        s.display.message = "descending in view until the tip starts to slide"
        r = s.touch(z_old - MAX_OVERTRAVEL_MM, FIRST_DESCENT_MM_S)
    else:
        print("Tip not visible - falling back to finding the dish blind (force backstop).")
        s.display.message = "tip not visible: finding the dish by force first"
        r = s.touch(z_old - MAX_OVERTRAVEL_MM, FIRST_DESCENT_MM_S)
        s.shift(0, 0, FIND_HOVER_MM, "lift to search height")
        d = spiral_search(s, rot, "just above the dish")
        if d is None:
            raise CalibrationFailedError("Could not find the tip anywhere near the field of view.")
        print(f"Tip found just above the dish at px {np.round(d.px, 1).tolist()} (score {d.score:.2f}).")
        s.servo(s.fov_centre, "centring", best_effort=True)
        r = s.touch(r["tip_z"] - MAX_OVERTRAVEL_MM, DESCENT_MM_S)
    print(f"Dish found: model tip z {r['tip_z']:.3f} (old calibrated dish {z_old:.3f}).")
    px = s.contact_px(r)
    if px is None:
        raise CalibrationFailedError("Tip not detected at contact.")
    s.display.marks.append((px[0], px[1], "lime", "o"))
    xy = s.tip(s.contact_q(r))[:2, 3].copy()
    s.shift(0, 0, HOVER_MM, "lift")
    return {"z_dish": r["tip_z"], "px": px, "rot": rot, "xy": xy}


def stage_tip_calibration(s: Session, first: dict) -> np.ndarray:
    s.display.phase = "3/5 tip calibration (same spot, different wrist angles)"
    ref_px = np.asarray(first["px"])
    rot0 = first["rot"]
    pip = rot0[:, 2]
    diag = np.array([1.0, 1.0, 0]) / np.sqrt(2)
    poses = [
        (np.array([1.0, 0, 0]), 0.0),
        (np.array([1.0, 0, 0]), PIVOT_TILTS_DEG),
        (np.array([1.0, 0, 0]), -PIVOT_TILTS_DEG),
        (np.array([0, 1.0, 0]), PIVOT_TILTS_DEG),
        (np.array([0, 1.0, 0]), -PIVOT_TILTS_DEG),
        (pip, PIVOT_TWIST_DEG),
        (pip, -PIVOT_TWIST_DEG),
        (diag, 0.7 * PIVOT_TILTS_DEG),
        (diag, -0.7 * PIVOT_TILTS_DEG),
    ]
    flanges, pix = [], []
    z_dish = first["z_dish"]
    ref_xy = first["xy"]
    z_safe, z_hover = z_dish + 3.0, z_dish + HOVER_MM  # ABSOLUTE heights over the measured dish
    for i, (axis, ang) in enumerate(poses):
        rot = axis_angle(axis, np.radians(ang)) @ rot0
        s.display.message = f"touch the same spot from wrist angle {i + 1} of {len(poses)} ({ang:+.0f} deg)"
        try:
            t = s.tip()
            s.move_tip([t[0, 3], t[1, 3], z_safe], t[:3, :3], "lift before rotating")
            s.move_tip([ref_xy[0], ref_xy[1], z_safe], rot, f"rotate {i + 1}")
            s.move_tip([ref_xy[0], ref_xy[1], z_hover], rot, "down to hover")
            s.servo(ref_px, f"orientation {i + 1}")
            # The tip model is the unknown here, so each orientation's dish height in the model
            # can differ by a mm or two - the over-travel floor allows for it.
            r = s.touch(z_dish - MAX_OVERTRAVEL_MM, DESCENT_MM_S)
        except CalibrationFailedError as e:
            print(f"  orientation {i + 1}: skipped ({e})")
            with contextlib.suppress(Exception):
                t = s.tip()
                s.move_tip([t[0, 3], t[1, 3], z_safe], t[:3, :3], "recover to safe height")
            continue
        px = s.contact_px(r)
        if px is None:
            print(f"  orientation {i + 1}: tip not detected at contact - skipped")
        else:
            flanges.append(s.kin.flange(s.contact_q(r)))
            pix.append(px)
            s.display.marks.append((px[0], px[1], "lime", "o"))
            print(f"  orientation {i + 1}: touch at px {np.round(px, 1).tolist()}")
        s.shift(0, 0, HOVER_MM, "lift")

    with contextlib.suppress(Exception):
        t = s.tip()
        s.move_tip([t[0, 3], t[1, 3], z_safe], t[:3, :3], "lift")
        s.move_tip([ref_xy[0], ref_xy[1], z_safe], rot0, "back to the reference orientation")
    if len(flanges) < MIN_PIVOT_TOUCHES:
        raise CalibrationFailedError(f"Only {len(flanges)} usable pivot touches (need {MIN_PIVOT_TOUCHES}).")
    # R_k t + p_k = c + e_k, with e_k the known image offset of touch k from the reference pixel.
    a = np.vstack([np.hstack([f[:3, :3], -np.eye(3)]) for f in flanges])
    e = [np.array([*(s.lin @ (np.asarray(p) - ref_px)), 0.0]) for p in pix]
    b = np.concatenate([ek - f[:3, 3] for ek, f in zip(e, flanges, strict=True)])
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    tip, c = sol[:3], sol[3:]
    res = np.array(
        [np.linalg.norm(f[:3, :3] @ tip + f[:3, 3] - c - ek) for f, ek in zip(flanges, e, strict=True)]
    )
    print(
        f"Pivot fit over {len(flanges)} touches: tip {np.round(tip, 2).tolist()} mm (FRF), "
        f"residual {np.sqrt(np.mean(res**2)):.3f} mm rms"
    )
    s.display.message = f"tip measured: residual {np.sqrt(np.mean(res**2)) * 1000:.0f} um"
    s.display.marks.clear()
    return tip


def save_tip_and_reimport(s: Session, tip: np.ndarray) -> None:
    path = s.bench.root / "bench.yaml"
    text = path.read_text(encoding="utf-8")
    value = f"[{tip[0]:.2f}, {tip[1]:.2f}, {tip[2]:.2f}]"
    text, n = re.subn(r"(?m)^  measured_tip_in_flange: .*$", f"  measured_tip_in_flange: {value}", text)
    if n != 1:
        raise CalibrationFailedError("Could not find tool.measured_tip_in_flange in bench.yaml.")
    path.write_text(text, encoding="utf-8", newline="\n")
    import import_bench_cad

    import_bench_cad.main()
    s.reload(BenchConfig.load(RIG))


def ring_points(centre, radius, n, frac, phase=0.0):
    return [
        centre + frac * radius * np.array([np.cos(a), np.sin(a)])
        for a in np.radians(np.arange(n) * 360.0 / n + phase)
    ]


def touch_at_pixel(s: Session, px, rot, z_dish: float, xy_of_px, label: str):
    """Hover over where ``xy_of_px`` says pixel ``px`` is, touch, detect. Returns (detection, q)."""
    xy = xy_of_px(px)
    s.display.overlays["target"] = (px[0], px[1], "yellow", "+")
    s.move_tip([xy[0], xy[1], z_dish + HOVER_MM], rot, label)
    r = s.touch(z_dish - MAX_OVERTRAVEL_MM, DESCENT_MM_S)
    px = s.contact_px(r)
    q = s.contact_q(r)
    s.shift(0, 0, HOVER_MM, "lift")
    return (None if px is None else SimpleNamespace(px=px)), q


def stage_scope_calibration(s: Session, rot) -> ScopeCalibration:
    s.display.phase = "4/5 scope calibration (grid of touches)"
    # Re-anchor the old affine at the current tip: one touch at the field centre.
    d, q = touch_at_pixel(
        s, s.fov_centre, rot, s.calib.surface_z_mm, lambda p: s.calib.pixel_to_xy([p])[0], "anchor"
    )
    if d is None:
        raise CalibrationFailedError("Tip not detected at the anchor touch.")
    t = s.kin.tip(q)
    anchor_px, anchor_xy, z_dish = np.asarray(d.px), t[:2, 3], t[2, 3]

    def xy_of_px(p):
        return anchor_xy + s.lin @ (np.asarray(p) - anchor_px)

    pts = ring_points(s.fov_centre, s.fov_radius, GRID_OUTER, 0.7) + ring_points(
        s.fov_centre, s.fov_radius, GRID_INNER, 0.35, 45
    )
    pixels, xys, zs = [anchor_px], [anchor_xy], [z_dish]
    for i, p in enumerate(pts):
        s.display.message = f"scope point {i + 1}/{len(pts)}"
        try:
            d, q = touch_at_pixel(s, p, rot, z_dish, xy_of_px, f"scope point {i + 1}")
        except CalibrationFailedError as e:
            print(f"  point {i + 1}: skipped ({e})")
            continue
        if d is None:
            print(f"  point {i + 1}: tip not detected - skipped")
            continue
        t = s.kin.tip(q)
        pixels.append(np.asarray(d.px))
        xys.append(t[:2, 3])
        zs.append(t[2, 3])
        s.display.marks.append((d.px[0], d.px[1], "lime", "o"))
    if len(pixels) < 6:
        raise CalibrationFailedError(f"Only {len(pixels)} scope points (need 6).")
    w, h = s.display.last_frame.shape[1], s.display.last_frame.shape[0]
    fit = ScopeCalibration.fit(pixels, xys, float(np.mean(zs)), (w, h))
    print(
        f"Scope fit over {len(pixels)} points: {fit.residual_px_rms:.2f} px rms = {fit.residual_um_rms:.1f} um, "
        f"{fit.mm_per_px * 1000:.2f} um/px, dish z {fit.surface_z_mm:.3f} (+/- {np.std(zs) * 1000:.0f} um)"
    )
    return fit


def stage_verify(s: Session, fit: ScopeCalibration, rot) -> list:
    s.display.phase = "5/5 verify"
    s.display.marks.clear()
    rng = np.random.default_rng()
    errs = []
    for i in range(VERIFY_POINTS):
        r = s.fov_radius * 0.6 * np.sqrt(rng.random())
        a = rng.uniform(0, 2 * np.pi)
        p = s.fov_centre + r * np.array([np.cos(a), np.sin(a)])
        s.display.message = f"verify {i + 1}/{VERIFY_POINTS}: does the tip land where predicted?"
        s.display.overlays["predicted"] = (p[0], p[1], "cyan", "+")
        d, _ = touch_at_pixel(
            s, p, rot, fit.surface_z_mm, lambda pp: fit.pixel_to_xy([pp])[0], f"verify {i + 1}"
        )
        if d is not None:
            e = float(np.linalg.norm(np.asarray(d.px) - p))
            errs.append(e)
            print(
                f"  verify {i + 1}: detected {e:.1f} px = {e * fit.mm_per_px * 1000:.0f} um from the prediction"
            )
    return errs


def main() -> None:
    import bota_driver

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    home = np.asarray(require_home_joints(RIG), dtype=float)

    view = LiveView(TITLE)  # window first: nothing connected if it cannot open
    sensor = bota_driver.BotaDriver(str(bota_config_path(RIG)))
    if not (sensor.configure() and sensor.tare() and sensor.activate()):
        view.close()
        sys.exit("Bota sensor failed to configure/tare/activate - check its COM port in the config.")
    stream = WrenchStream(sensor)
    detector = ContactDetector(
        stream, ContactThresholds(torque_mnm=BACKSTOP_TORQUE_MNM, force_limit_n=BACKSTOP_FORCE_N)
    )
    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=False))
    robot.connect()
    robot.configure()
    display = Display(view, robot, detector, bench.camera_key, TITLE)
    s = Session(robot, display, bench, detector)
    ok = False
    try:
        make_ready(s.meca)
        s.meca.SetCartLinVel(CART_SPEED_MM_S)
        s.meca.SetJointVel(JOINT_VEL_DEG_S)
        s.meca.SetVelTimeout(0.3)
        from pathlib import Path

        s.debug_dir = Path(DEBUG_DIR_ROOT) / time.strftime("%Y%m%d-%H%M%S")
        s.debug_dir.mkdir(parents=True, exist_ok=True)
        print(f"Debug images: {s.debug_dir}")
        stage_home_and_background(s, home)
        first = stage_find_tip(s, home)
        tip = stage_tip_calibration(s, first)
        save_tip_and_reimport(s, tip)
        fit = stage_scope_calibration(s, first["rot"])
        cal_path = s.bench.calibration_path()
        if cal_path.exists():
            backup = cal_path.with_name(f"scope_calibration.{time.strftime('%Y%m%d-%H%M%S')}.bak.json")
            shutil.copy(cal_path, backup)
            print(f"Previous calibration backed up to {backup.name}")
        fit.save(cal_path)
        s.reload(BenchConfig.load(RIG))
        errs = stage_verify(s, fit, first["rot"])
        ok = True
        s.display.phase = "done - returning home"
        if errs:
            s.display.message = (
                f"verify: median {np.median(errs):.1f} px = {np.median(errs) * fit.mm_per_px * 1000:.0f} um"
            )
        try:
            run_joint_path(s.meca, s.planner.plan_transit(s.q(), home, retreat=True), s.display)
        except PlanningError as e:
            print(f"Could not plan the way home ({e}); the arm holds its pose.")
        print("\nAUTO-CALIBRATION COMPLETE")
        print(f"  tip (FRF): {np.round(tip, 2).tolist()} mm -> bench.yaml tool.measured_tip_in_flange")
        print(
            f"  scope: {fit.residual_px_rms:.2f} px rms ({fit.residual_um_rms:.0f} um), dish z {fit.surface_z_mm:.3f}"
        )
        if errs:
            print(f"  verify: {len(errs)} touches, median {np.median(errs):.1f} px, max {max(errs):.1f} px")
    except AbortedError:
        print("Aborted by the user - the arm stops and holds its pose.")
        with contextlib.suppress(Exception):
            s.meca.ClearMotion()
    except CalibrationFailedError as e:
        print(f"\nCALIBRATION FAILED: {e}\nThe arm stops and holds its pose; nothing further was saved.")
        with contextlib.suppress(Exception):
            s.meca.ClearMotion()
    finally:
        with contextlib.suppress(Exception):
            s.meca.MoveLinVelWrf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        view.close()
        stream.stop()
        with contextlib.suppress(Exception):
            sensor.deactivate()
            sensor.shutdown()
        robot.disconnect()
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
