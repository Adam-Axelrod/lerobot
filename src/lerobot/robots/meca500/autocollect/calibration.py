"""Microscope pixel <-> robot XY at the dish surface, for one bench.

Fitted from pairs of (tip pixel clicked in the microscope image, tip XY from ``GetPose()``),
all taken with the tip touching the dish. A full affine map is used on purpose: it absorbs
the Basler's sensor-side mirroring, its 270 deg rotation and the bench's 180 deg base turn
without anyone having to get those signs right by hand.

The residual of that fit is the label noise of every auto-collected demo - the gap between
where the marker is drawn and where the tip actually ends up - so it is stored with the
calibration and checked before collection starts.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np


def fit_affine(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares 2x3 affine ``A`` with ``dst ~= A @ [src, 1]``. Needs >= 3 non-collinear points."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if len(src) < 3:
        raise ValueError(f"need at least 3 point pairs, got {len(src)}")
    x = np.hstack([src, np.ones((len(src), 1))])
    sol, *_ = np.linalg.lstsq(x, dst, rcond=None)
    return sol.T


def apply_affine(a: np.ndarray, pts) -> np.ndarray:
    pts = np.atleast_2d(np.asarray(pts, float))
    return pts @ a[:, :2].T + a[:, 2]


def invert_affine(a: np.ndarray) -> np.ndarray:
    lin_inv = np.linalg.inv(a[:, :2])
    return np.hstack([lin_inv, (-lin_inv @ a[:, 2])[:, None]])


@dataclass
class ScopeCalibration:
    px_to_xy: list[list[float]]  # 2x3 affine, pixels -> base-frame mm
    surface_z_mm: float  # tip Z (base frame) when touching the dish
    image_size: list[int]  # [width, height] of the frame the pixels refer to
    residual_px_rms: float
    residual_um_rms: float
    n_points: int
    pairs: list = field(default_factory=list)  # [[u, v, x, y], ...] kept for re-fitting

    @classmethod
    def fit(cls, pixels, xy, surface_z_mm: float, image_size) -> ScopeCalibration:
        pixels, xy = np.asarray(pixels, float), np.asarray(xy, float)
        a = fit_affine(pixels, xy)
        xy_res = np.linalg.norm(apply_affine(a, pixels) - xy, axis=1)
        px_res = np.linalg.norm(apply_affine(invert_affine(a), xy) - pixels, axis=1)
        return cls(
            px_to_xy=a.tolist(),
            surface_z_mm=float(surface_z_mm),
            image_size=[int(image_size[0]), int(image_size[1])],
            residual_px_rms=float(np.sqrt(np.mean(px_res**2))),
            residual_um_rms=float(np.sqrt(np.mean(xy_res**2)) * 1000),
            n_points=len(pixels),
            pairs=np.hstack([pixels, xy]).tolist(),
        )

    @property
    def mm_per_px(self) -> float:
        """Mean scale, for reporting errors in physical units."""
        return float(np.sqrt(abs(np.linalg.det(np.asarray(self.px_to_xy)[:, :2]))))

    def pixel_to_xy(self, px) -> np.ndarray:
        return apply_affine(np.asarray(self.px_to_xy), px)

    def xy_to_pixel(self, xy) -> np.ndarray:
        return apply_affine(invert_affine(np.asarray(self.px_to_xy)), xy)

    def in_image(self, px, margin_px: float = 0.0) -> np.ndarray:
        px = np.atleast_2d(px)
        w, h = self.image_size
        return (
            (px[:, 0] >= margin_px)
            & (px[:, 0] <= w - 1 - margin_px)
            & (px[:, 1] >= margin_px)
            & (px[:, 1] <= h - 1 - margin_px)
        )

    def save(self, path: Path) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8", newline="\n")

    @classmethod
    def load(cls, path: Path) -> ScopeCalibration:
        return cls(**json.loads(Path(path).read_text()))
