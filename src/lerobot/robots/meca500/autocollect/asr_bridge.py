"""Huanyu's ActionStateRuntime (ASR) on the Meca500/Basler bench: tip detection and visual servoing.

ASR (github.com/RViMLab/ActionStateRuntime) is an optional dependency, imported only here and
only when a function needs it. Its perception package (ONNX, ViT-B) finds the pipette tip in a
microscope frame; its reference loop servos the tip onto a target pixel through a host adapter
that moves the tip by small robot-frame deltas. This module supplies:

    AsrTipDetector   the perception package as a tip detector (frame -> tip pixel + confidence),
                     limited to the lit field, where the tip can be seen at all
    TipWatcher       runs the detector on the latest frame in a background thread while a policy
                     drives, and says when the tip has been seen confidently (the handover cue)
    Meca500AsrHost   the ASRHost adapter: snapshots (frame + forward-kinematics tip, metres) and
                     tip deltas carried out as collision-checked straight lines with a press limit
    make_service     an ASRService for the bench, with the convergence threshold set
    VirtualSlide     a robot base displaced by a fixed offset, in software (the slide test)

Frames from the robot are RGB; ASR is given BGR, as its own tools read images with OpenCV.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from .planner import Planner, PlanningError

# Converge on the target to within this many pixels (13.3 um/px: 3 px = 40 um). ASR's default
# is 30 px, set for a 4x-objective cell-picking task; at our scale that is 0.4 mm.
DEFAULT_CONVERGE_PX = 3.0


@dataclass
class TipSighting:
    px: tuple[float, float]
    confidence: float
    t: float  # time.perf_counter() of the frame


class AsrTipDetector:
    """ASR's perception package as a tip detector.

    ``runtime`` is an ``action_state_runtime.runtime.ActionStateRuntime`` with the real backend
    (for example ``ASRService.runtime`` after ``initialize``). Candidates outside the lit field
    (``field_centre_px``, ``field_radius_px`` + ``field_margin_px``) are dropped: nothing there is
    lit well enough to be the tip.
    """

    def __init__(self, runtime, field_centre_px, field_radius_px: float, field_margin_px: float = 10.0):
        self.runtime = runtime
        self.centre = np.asarray(field_centre_px, dtype=float)
        self.radius = float(field_radius_px) + float(field_margin_px)
        self._lock = threading.Lock()

    def detect(self, rgb: np.ndarray) -> TipSighting | None:
        bgr = np.ascontiguousarray(rgb[..., ::-1])
        t = time.perf_counter()
        with self._lock:  # one inference at a time, whichever thread asks
            record = self.runtime.infer_affordances(image=bgr, phase="pickup", save_json=False)
        best = None
        for c in (record.get("affordances") or {}).get("tip_candidates") or []:
            xy = c.get("xy")
            conf = c.get("confidence")
            if xy is None or conf is None:
                continue
            if np.linalg.norm(np.asarray(xy[:2], dtype=float) - self.centre) > self.radius:
                continue
            if best is None or conf > best.confidence:
                best = TipSighting((float(xy[0]), float(xy[1])), float(conf), t)
        return best


class TipWatcher:
    """Runs a detector on the newest frame, over and over, in a background thread.

    ``frame_source()`` returns the newest RGB frame (or None). ``seen`` is set once ``hits``
    sightings in a row reach ``min_confidence`` and lie within ``max_jump_px`` of each other - the
    "visibility guard" for the handover. A single confident frame is not enough: the dish has
    debris that can briefly look like a tip.
    """

    def __init__(
        self,
        detector: AsrTipDetector,
        frame_source: Callable[[], np.ndarray | None],
        min_confidence: float = 0.8,
        hits: int = 2,
        max_jump_px: float = 40.0,
    ):
        self.detector = detector
        self.frame_source = frame_source
        self.min_confidence = min_confidence
        self.hits = hits
        self.max_jump_px = max_jump_px
        self.seen = threading.Event()
        self.sighting: TipSighting | None = None
        self.history: list[dict] = []
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t0 = 0.0

    def start(self) -> None:
        self.seen.clear()
        self._stop.clear()
        self.sighting, self.history, self.error = None, [], None
        self._t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._run, name="tip-watcher", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def _run(self) -> None:
        streak: list[TipSighting] = []
        try:
            while not self._stop.is_set():
                frame = self.frame_source()
                if frame is None:
                    time.sleep(0.02)
                    continue
                s = self.detector.detect(frame)
                self.history.append(
                    {
                        "t": round(time.perf_counter() - self._t0, 3),
                        "px": None if s is None else [round(v, 1) for v in s.px],
                        "confidence": None if s is None else round(s.confidence, 3),
                    }
                )
                if s is None or s.confidence < self.min_confidence:
                    streak = []
                    continue
                if streak and np.hypot(*(np.subtract(s.px, streak[-1].px))) > self.max_jump_px:
                    streak = []
                streak.append(s)
                if len(streak) >= self.hits:
                    self.sighting = s
                    self.seen.set()
                    return
        except Exception as e:  # reported, never raised into the control loop
            self.error = f"{type(e).__name__}: {e}"
            logging.exception("Tip watcher failed")


class Meca500AsrHost:
    """The ASR host contract on the Meca500.

    ``frame_source()`` returns the newest RGB camera frame. ``joints()`` returns the measured joint
    angles (deg). ``move(path)`` carries out a collision-checked joint path and returns once the
    arm has settled. Every tip delta ASR asks for becomes a straight, orientation-holding line from
    the current pose, planned and collision-checked by ``planner``; it is refused (not clipped) if
    it is longer than ``max_step_mm``, leaves the ``max_travel_mm`` box around where the servo
    started, or would put the tip below the dish surface by more than ``press_limit_mm``.
    ``surface_z_mm(xy)`` gives the dish surface under a tip position.
    """

    is_mock = False

    def __init__(
        self,
        planner: Planner,
        frame_source: Callable[[], np.ndarray],
        joints: Callable[[], np.ndarray],
        move: Callable[[np.ndarray], None],
        surface_z_mm: Callable[[np.ndarray], float],
        speed_mm_s: float = 0.5,
        max_step_mm: float = 1.0,
        max_travel_mm: float = 3.0,
        press_limit_mm: float = 0.1,
    ):
        self.planner = planner
        self.kin = planner.kin
        self.frame_source = frame_source
        self.joints = joints
        self.move = move
        self.surface_z_mm = surface_z_mm
        self.speed_mm_s = speed_mm_s
        self.max_step_mm = max_step_mm
        self.max_travel_mm = max_travel_mm
        self.press_limit_mm = press_limit_mm
        self.origin_mm: np.ndarray | None = None
        self.moves: list[dict] = []
        self.estimates: list[dict] = []

    def reset(self) -> None:
        """Call before each servo: sets the travel box around the current tip."""
        self.origin_mm = self.kin.tip(self.joints())[:3, 3].copy()
        self.moves, self.estimates = [], []

    # ------------------------------------------------------------------ ASRHost

    def capture_snapshot(self, request):
        from action_state_runtime.contracts import SensorSnapshot, SnapshotResult

        frame = self.frame_source()
        if frame is None:
            return SnapshotResult(False, "frame_unavailable")
        q = np.asarray(self.joints(), dtype=float)
        t = self.kin.tip(q)
        p = t[:3, 3]
        height = p[2] - self.surface_z_mm(p[:2])
        h, w = frame.shape[:2]
        snap = SensorSnapshot(
            frame=np.ascontiguousarray(frame[..., ::-1]),
            image_size=(int(w), int(h)),
            joints_rad=tuple(float(v) for v in np.radians(q)),
            tip_position_robot_m=tuple(float(v) / 1000 for v in p),
            tool_axis_robot=tuple(float(v) for v in t[:3, 2]),
            depth_m=float(height) / 1000,
        )
        return SnapshotResult(True, "ok", snap)

    def execute_tip_delta(self, command, cancel_token):
        from action_state_runtime.contracts import TipMotionResult

        d_mm = np.asarray(command.delta_tip_robot_m, dtype=float) * 1000
        q0 = np.asarray(self.joints(), dtype=float)
        p0 = self.kin.tip(q0)[:3, 3].copy()
        requested = tuple(float(v) for v in command.delta_tip_robot_m)

        def refuse(reason: str):
            self.moves.append({"requested_um": np.round(d_mm * 1000, 1).tolist(), "refused": reason})
            return TipMotionResult(False, reason, requested, (0.0, 0.0, 0.0), tuple(p0 / 1000), tuple(p0 / 1000))

        if cancel_token.cancelled:
            return refuse("canceled")
        if self.origin_mm is None:
            self.reset()
        if np.linalg.norm(d_mm) > self.max_step_mm:
            return refuse(f"step {np.linalg.norm(d_mm):.2f} mm > {self.max_step_mm} mm")
        p1 = p0 + d_mm
        if np.abs(p1[:2] - self.origin_mm[:2]).max() > self.max_travel_mm:
            return refuse(f"leaves the {self.max_travel_mm} mm servo box")
        if p1[2] - self.surface_z_mm(p1[:2]) < -self.press_limit_mm:
            return refuse("below the dish surface")
        if np.linalg.norm(d_mm) < 1e-3:
            q1 = q0
        else:
            try:
                path = self.planner.plan_line(q0, p1, self.speed_mm_s, "asr servo step")
            except PlanningError as e:
                return refuse(str(e))
            self.move(path)
            q1 = np.asarray(self.joints(), dtype=float)
        p_after = self.kin.tip(q1)[:3, 3]
        self.moves.append(
            {
                "requested_um": np.round(d_mm * 1000, 1).tolist(),
                "actual_um": np.round((p_after - p0) * 1000, 1).tolist(),
            }
        )
        return TipMotionResult(
            True, "ok", requested, tuple((p_after - p0) / 1000), tuple(p0 / 1000), tuple(p_after / 1000)
        )

    def restore_robot_control(self):
        from action_state_runtime.contracts import RestoreControlResult

        return RestoreControlResult(True)  # no admittance mode on this arm: nothing to restore

    def publish_tip_estimate(self, estimate) -> None:
        self.estimates.append(
            {
                "px": [round(float(v), 1) for v in estimate.tip_px],
                "source": estimate.source,
                "confidence": estimate.confidence,
                "prediction_only": estimate.prediction_only,
            }
        )


def make_service(
    host,
    site_yaml: str | Path,
    output_dir: str | Path,
    converge_px: float = DEFAULT_CONVERGE_PX,
    max_steps: int = 8,
    settle_s: float = 0.3,
):
    """An ASRService for the bench, with ``converge_px`` in place of ASR's 30 px.

    The controller builds its loop configuration from built-in defaults, not from the site yaml,
    so the threshold is set on that configuration directly. The protection radius and the
    worsening margin are scaled with it, as in ASR's defaults (80 px and 5 px at 30 px).
    """
    from action_state_runtime.config.runtime import load_config
    from action_state_runtime.contracts import ASRAnchors
    from action_state_runtime.service import ASRService

    if not Path(site_yaml).is_file():
        # load_config falls back to ASR's packaged defaults (another perception package, another
        # camera's Jacobian) for a missing file: refuse instead.
        raise FileNotFoundError(f"ASR site file {site_yaml} not found")
    cfg = load_config(str(site_yaml))
    service = ASRService(
        host,
        anchors=ASRAnchors(),
        output_dir=str(output_dir),
        runtime_config=cfg,
        max_steps=max_steps,
        settle_s=settle_s,
    )
    scale = converge_px / 30.0
    lc = service._controller._loop_config
    service._controller._loop_config = replace(
        lc,
        step=replace(lc.step, converge_px=converge_px, target_protection_radius_px=80.0 * scale),
        exit=replace(
            lc.exit,
            converge_px=converge_px,
            target_protection_radius_px=80.0 * scale,
            worsen_margin_px=5.0 * scale,
        ),
    )
    return service


class VirtualSlide:
    """The robot base displaced by ``offset_mm`` (robot frame), in software.

    A real slide moves the base relative to the microscope: the same joint angles then put the tip
    ``offset_mm`` away from where they used to. Here the arm stays put; instead the policy is shown
    "virtual" joints - those that would put the tip where it really is if the base had moved - and
    each virtual command is carried out at the real joints that put the tip ``offset_mm`` from
    where the virtual joints would. The tip pose, the camera image and the contact are real;
    only the arm's links sit slightly differently, and every real command is still collision-
    checked against the real scene.
    """

    def __init__(self, kin, offset_mm):
        self.kin = kin
        self.offset = np.zeros(3)
        self.offset[: len(offset_mm)] = np.asarray(offset_mm, dtype=float)

    @property
    def active(self) -> bool:
        return bool(np.any(self.offset != 0))

    def _move(self, q, d) -> np.ndarray | None:
        q = np.asarray(q, dtype=float)
        if not np.any(d):
            return q
        pose = self.kin.tip(q).copy()
        pose[:3, 3] += d
        return self.kin.ik(pose, q)

    def to_real(self, q_virtual) -> np.ndarray | None:
        return self._move(q_virtual, self.offset)

    def to_virtual(self, q_real) -> np.ndarray | None:
        return self._move(q_real, -self.offset)
