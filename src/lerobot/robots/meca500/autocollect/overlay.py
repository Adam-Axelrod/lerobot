"""The digital target drawn onto the microscope frame.

One drawing function used everywhere - live recording, the rollout, and any offline preview -
so a policy is never trained on one marker and deployed against a subtly different one.

``TargetOverlayStep`` plugs into a robot-observation processor pipeline, which in
``record_loop`` runs between ``robot.get_observation()`` and the dataset/display, so what is
recorded is exactly what is shown and what a policy later sees.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import cv2
import numpy as np

from lerobot.processor.pipeline import ObservationProcessorStep


@dataclass
class TargetStyle:
    """High-contrast ring + crosshair. Colour is in the frame's channel order (RGB here)."""

    radius_px: int = 14
    thickness_px: int = 2
    cross_px: int = 6  # half-length of the centre cross; 0 disables it
    color: tuple[int, int, int] = (0, 255, 0)
    outline_color: tuple[int, int, int] | None = (0, 0, 0)  # thin dark edge so it reads on bright fields


def draw_target(img: np.ndarray, px, style: TargetStyle | None = None) -> np.ndarray:
    """Return a copy of ``img`` (HxWx3 uint8) with the target centred at pixel ``px = (u, v)``."""
    style = style or TargetStyle()
    out = np.ascontiguousarray(img).copy()
    # Sub-pixel centre via cv2's fixed-point shift, so the drawn centre is where px says.
    shift = 4
    scale = 1 << shift
    c = (int(round(px[0] * scale)), int(round(px[1] * scale)))
    r = int(round(style.radius_px * scale))
    k = int(round(style.cross_px * scale))

    def _draw(color, extra):
        t = style.thickness_px + extra
        cv2.circle(out, c, r, color, t, cv2.LINE_AA, shift)
        if style.cross_px > 0:
            cv2.line(out, (c[0] - k, c[1]), (c[0] + k, c[1]), color, t, cv2.LINE_AA, shift)
            cv2.line(out, (c[0], c[1] - k), (c[0], c[1] + k), color, t, cv2.LINE_AA, shift)

    if style.outline_color is not None:
        _draw(style.outline_color, 2)
    _draw(style.color, 0)
    return out


@dataclass
class TargetOverlayStep(ObservationProcessorStep):
    """Draws the current target onto ``camera_key``. ``target_px = None`` draws nothing.

    With a ``sampler``, every ``reset()`` - which the rollout's episodic strategy calls at the
    start of each episode - draws a fresh target from it, so evaluation episodes get random
    targets the same way recording does. The recorder sets targets explicitly instead.
    """

    camera_key: str = "microscope_cam"
    target_px: tuple[float, float] | None = None
    style: TargetStyle = field(default_factory=TargetStyle)
    sampler: Callable[[], tuple[float, float]] | None = None

    def set_target(self, px) -> None:
        self.target_px = None if px is None else (float(px[0]), float(px[1]))

    def reset(self) -> None:
        if self.sampler is not None:
            self.set_target(self.sampler())

    def observation(self, observation):
        if self.target_px is not None and self.camera_key in observation:
            observation[self.camera_key] = draw_target(
                observation[self.camera_key], self.target_px, self.style
            )
        return observation

    def transform_features(self, features):
        return features

    def get_config(self):
        return {"camera_key": self.camera_key, "style": self.style.__dict__}
