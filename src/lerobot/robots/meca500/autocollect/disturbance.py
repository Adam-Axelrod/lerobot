"""A simulated camera knock: the microscope image translated by a known number of pixels.

A real knock is unrepeatable and costs a recalibration. Shifting every frame is exactly what a
small camera move looks like from software (the dish, the tip and the lit circle all move
together; the digital target stays at its pixel), and nothing physical changes, so the collision
model and the dish height stay valid and the arm is no less safe.
"""

from __future__ import annotations

import numpy as np


def shift_frame(frame: np.ndarray, dx: int, dy: int) -> np.ndarray:
    """``frame`` with its content moved ``dx`` right and ``dy`` down; the exposed edge is black."""
    if dx == 0 and dy == 0:
        return frame
    h, w = frame.shape[:2]
    out = np.zeros_like(frame)
    x0, y0 = max(dx, 0), max(dy, 0)
    x1, y1 = min(w + dx, w), min(h + dy, h)
    if x1 > x0 and y1 > y0:
        out[y0:y1, x0:x1] = frame[y0 - dy : y1 - dy, x0 - dx : x1 - dx]
    return out


def sample_shift(
    seed: int, magnitude_px: float, target_px, field_centre_px, field_radius_px
) -> tuple[int, int]:
    """A whole-pixel shift of about ``magnitude_px`` in a direction fixed by ``seed``.

    The direction is redrawn until the target still lies in the shifted lit field. Outside it the
    tip detector cannot see the tip, so the episode could not be scored. Moving towards the target
    always qualifies, so one exists for any target already inside the field.
    """
    if magnitude_px <= 0:
        return 0, 0
    target, centre = np.asarray(target_px, float), np.asarray(field_centre_px, float)
    rng = np.random.default_rng(seed + 9_000_000)  # independent of the path and stage draws
    for _ in range(200):
        angle = rng.uniform(0, 2 * np.pi)
        shift = np.round(magnitude_px * np.array([np.cos(angle), np.sin(angle)]))
        if np.linalg.norm(target - (centre + shift)) <= field_radius_px:
            return int(shift[0]), int(shift[1])
    towards = (target - centre) / max(float(np.linalg.norm(target - centre)), 1e-9)
    shift = np.round(magnitude_px * towards)
    return int(shift[0]), int(shift[1])
