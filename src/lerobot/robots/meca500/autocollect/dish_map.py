"""Dish surface height as a function of the stage position: a plane fitted to touches.

The scope calibration measures the dish height at one stage position. The dish and its acrylic
plate are not perfectly level, so moving the stage +-10 mm changes the height under the tip: a
0.5 deg tilt is ~90 um, and the recorded touch has no force guard. auto_calibrate touches the
dish at several stage positions and fits z = z0 + gx (X - X0) + gy (Y - Y0); record_auto touches
at that height for each episode's stage position.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

OUTLIER_UM = 40.0  # a touch this far off the plane of the others is dropped


@dataclass
class DishHeightMap:
    reference_xy: list[float]  # stage (X, Y) mm the plane is expanded about
    plane: list[float]  # [z0, gx, gy]: robot-frame tip z (mm) at the reference, slopes per stage mm
    stage_xy: list[list[float]]  # the touches it was fitted to
    z: list[float]
    residual_um_rms: float
    rejected: list[int] = field(default_factory=list)  # indices into stage_xy/z left out as outliers

    @classmethod
    def fit(cls, stage_xy, z, reference_xy, outlier_um: float = OUTLIER_UM, max_rejected: int = 2) -> DishHeightMap:
        """Least-squares plane; then the worst touch is dropped while it is more than ``outlier_um``
        off the plane of the others (a late or early slide detection: 1 of 8 sim touches was 96 um
        off, the rest within 20)."""
        xy, z = np.asarray(stage_xy, float), np.asarray(z, float)
        if len(z) < 3:
            raise ValueError("A plane needs at least 3 touches.")
        a = np.column_stack([np.ones(len(z)), xy - np.asarray(reference_xy, float)])
        keep = np.ones(len(z), bool)
        while True:
            coef, *_ = np.linalg.lstsq(a[keep], z[keep], rcond=None)
            res = z - a @ coef
            worst = int(np.argmax(np.where(keep, np.abs(res), -1)))
            if keep.sum() - 1 < 4 or (~keep).sum() >= max_rejected or abs(res[worst]) * 1000 <= outlier_um:
                break
            keep[worst] = False
        return cls(
            reference_xy=[float(v) for v in reference_xy],
            plane=[float(v) for v in coef],
            stage_xy=xy.tolist(),
            z=z.tolist(),
            residual_um_rms=float(1000 * np.sqrt(np.mean(res[keep] ** 2))),
            rejected=[int(i) for i in np.flatnonzero(~keep)],
        )

    def surface_z(self, stage_xy) -> float:
        z0, gx, gy = self.plane
        d = np.asarray(stage_xy, float) - np.asarray(self.reference_xy, float)
        return float(z0 + gx * d[0] + gy * d[1])

    @property
    def tilt_deg(self) -> float:
        return float(np.degrees(np.arctan(np.hypot(self.plane[1], self.plane[2]))))

    def max_change_um(self, centre_xy, radius_mm: float) -> float:
        """Largest height difference from the reference over a disc of stage positions."""
        slope = np.hypot(self.plane[1], self.plane[2])
        reach = np.linalg.norm(np.asarray(centre_xy, float) - np.asarray(self.reference_xy, float)) + radius_mm
        return float(1000 * slope * reach)

    def save(self, path) -> None:
        Path(path).write_text(json.dumps(self.__dict__, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path) -> DishHeightMap:
        return cls(**json.loads(Path(path).read_text(encoding="utf-8")))
