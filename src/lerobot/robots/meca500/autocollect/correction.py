"""Vision correction of the final approach: detect the tip, move the touch point to cancel the error.

The open-loop plan lands the tip 60-120 um from the target (2026-11-03 touch tests), against a
50 um success threshold. The calibration's verify touches land within 1 px because they approach
from almost the calibration's own pose; a recorded episode comes in along a random path and
orientation, and small model errors add up.

So the recorder stops at checkpoints on the straight final approach, detects the tip, and shifts
the rest of the approach sideways (in the dish plane) by the pixel error mapped through the
scope calibration. The detected tip of a slightly raised, slightly blurred tip sits a few px
forward of where it will touch, so the last checkpoint is close to the dish.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .calibration import ScopeCalibration
from .tip_detect import detect_tip

# Weaker detections are ignored. Lower than the slide detector's 0.3: real in-focus touches
# scored 0.25-0.62 (2026-11-03), and here a false detection is caught by MAX_CORRECTION_MM, since
# the tip is already within ~10 px of the target, where the slide detector looks over the whole field.
MIN_SCORE = 0.15
MAX_CORRECTION_MM = 0.5  # a larger error means a bad detection or a bad calibration: don't act


@dataclass
class Correction:
    detected_px: tuple[float, float] | None
    error_px: float | None  # |expected - detected|
    shift_mm: tuple[float, float]  # applied in the dish plane (robot x, y); zero if skipped
    note: str = ""

    def meta(self) -> dict:
        return {
            "detected_px": None if self.detected_px is None else [round(float(v), 2) for v in self.detected_px],
            "error_px": None if self.error_px is None else round(float(self.error_px), 2),
            "shift_um": [round(1000 * float(v), 1) for v in self.shift_mm],
            "note": self.note,
        }


def px_shift_to_mm(calib: ScopeCalibration, d_px) -> np.ndarray:
    """A pixel displacement as a robot (x, y) displacement, through the calibration's linear part."""
    return np.asarray(calib.px_to_xy, float)[:, :2] @ np.asarray(d_px, float)


def measure(frame, background, expected_px, calib: ScopeCalibration, gain: float = 1.0) -> Correction:
    """Detect the tip in ``frame``; return the dish-plane shift that moves it onto ``expected_px``.

    ``expected_px`` is where the model puts the tip NOW (the calibration applied to the model tip),
    not the target: at a checkpoint the tip is still 1-4 mm back along the approach, 50-200 px
    from the target. The difference is the model error, and shifting the rest of the approach by
    it lands the touch on the target. For the final error, pass the target itself.
    """
    det = detect_tip(frame, background)
    if det is None or det.score < MIN_SCORE:
        note = "tip not detected" if det is None else f"weak detection (score {det.score:.2f})"
        return Correction(None if det is None else det.px, None, (0.0, 0.0), note)
    d_px = np.asarray(expected_px, float) - np.asarray(det.px, float)
    shift = gain * px_shift_to_mm(calib, d_px)
    err = float(np.linalg.norm(d_px))
    if np.linalg.norm(shift) > MAX_CORRECTION_MM:
        return Correction(det.px, err, (0.0, 0.0), f"correction {np.linalg.norm(shift):.2f} mm too large: skipped")
    return Correction(det.px, err, (float(shift[0]), float(shift[1])))
