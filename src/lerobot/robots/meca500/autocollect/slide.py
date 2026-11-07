"""The physical-slide test: which calibration the methods use, and which the measurement uses.

The robot is slid on the table relative to the microscope (scripts/meca500/apply_slide.py). The
methods under test must not be recalibrated in-plane by this, or the test shows nothing: they keep
the pixel -> robot XY map from BEFORE the slide (PINNED_CALIBRATION). The dish HEIGHT is not
pinned: every method takes it from the current dish height map and calibration, as after the
routine height check at the start of every session (decided with the paper session 2026-11-07:
the slide tests in-plane registration; the height check is routine and drifts by tens of um
between checks anyway). The post-slide calibration is otherwise used only to measure the slide,
to move the obstacles (registration.json) and to measure where the tip really ended.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, replace

from .bench import BenchConfig
from .calibration import ScopeCalibration

MEASUREMENT = "slide_measurement.json"
PINNED_CALIBRATION = "scope_calibration.pinned.json"


class PinnedXYBench(BenchConfig):
    """The bench as the methods see it after a slide: pre-slide pixel <-> XY map, current height."""

    def calibration(self) -> ScopeCalibration:
        current = super().calibration()
        pinned = ScopeCalibration.load(self.root / PINNED_CALIBRATION)
        return replace(
            current,
            px_to_xy=pinned.px_to_xy,
            residual_px_rms=pinned.residual_px_rms,
            residual_um_rms=pinned.residual_um_rms,
            pairs=pinned.pairs,
        )


class SoftwareSlideBench(BenchConfig):
    """The bench as the methods would see it had the base slid by ``offset_mm`` (eval_auto BASE_OFFSET_MM).

    After a physical slide d with the pre-slide map pinned, the methods take every pixel to the
    robot XY it had before, which in the slid robot's frame is d away from where that pixel really
    is. Here nothing moves; the methods' map is the current one shifted by d, so a command aimed
    at a pixel lands d off it, as it would. The scene, the dish model (placed by the map's linear
    part only, autocollect/stage.py) and the heights stay real, so every collision and press check
    still acts on the real arm in the real scene."""

    offset_mm: tuple[float, float] = (0.0, 0.0)

    def calibration(self) -> ScopeCalibration:
        current = super().calibration()
        a = [list(row) for row in current.px_to_xy]
        a[0][2] += float(self.offset_mm[0])
        a[1][2] += float(self.offset_mm[1])
        return replace(current, px_to_xy=a)


def software_slide_benches(bench: BenchConfig, offset_mm) -> SlideBenches:
    """SlideBenches for a software slide; refuses while a physical slide is in force."""
    if split_benches(bench).measurement is not None:
        raise RuntimeError("a physical slide is in force (apply_slide.py): a software slide on top of it is refused.")
    method = SoftwareSlideBench(rig=bench.rig, root=bench.root, raw=bench.raw)
    method.offset_mm = (float(offset_mm[0]), float(offset_mm[1]))
    d = list(method.offset_mm)
    measurement = {
        "software_slide": True,
        "slide_mm": d,
        "slide_norm_mm": round(float(sum(v * v for v in d) ** 0.5), 4),
        "rotation_deg": 0.0,
        "methods_use": "the current calibration with its pixel -> XY map shifted by slide_mm; the policy also sees virtual joints (VirtualSlide)",
    }
    return SlideBenches(method, bench, measurement)


@dataclass
class SlideBenches:
    method: BenchConfig  # what every method plans and aims with
    truth: BenchConfig  # what the final measurement uses (the current calibration)
    measurement: dict | None  # slide_measurement.json, None at the original position


def split_benches(bench: BenchConfig) -> SlideBenches:
    """The method and truth views of the bench; raises if the slide files are inconsistent."""
    root = bench.root
    m_path, cal = root / MEASUREMENT, root / PINNED_CALIBRATION
    reg_active = bench.registration_transform() is not None
    if not m_path.exists():
        if cal.exists() or reg_active:
            raise RuntimeError(
                f"{root}: {PINNED_CALIBRATION} or registration.json without {MEASUREMENT}. "
                "Run scripts/meca500/apply_slide.py --reset (back at the original position) or redo the slide measurement."
            )
        return SlideBenches(bench, bench, None)
    if not (cal.exists() and reg_active):
        raise RuntimeError(
            f"{root}: {MEASUREMENT} without the pinned pre-slide calibration and registration.json. "
            "Redo scripts/meca500/apply_slide.py."
        )
    return SlideBenches(
        PinnedXYBench(rig=bench.rig, root=bench.root, raw=bench.raw),
        bench,
        json.loads(m_path.read_text(encoding="utf-8")),
    )
