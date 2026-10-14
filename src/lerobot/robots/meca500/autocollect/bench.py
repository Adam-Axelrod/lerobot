"""Per-bench configuration for automated collection, loaded from ``assets/benches/<rig>/bench.yaml``.

Everything a bench needs that is NOT code lives in that folder: the obstacle STLs exported
from Fusion 360, the tool mesh, the scope calibration and the randomisation limits. Adding a
bench is a new folder, never a forked script - same rule as ``scripts/meca500/rigs.py``.

Units: mm and degrees, in the robot's base reference frame (BRF), throughout.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .calibration import ScopeCalibration
from .collision import BoxField, CollisionChecker, CylinderField, GridField, RobotPoints, TransformedField
from .kinematics import REPO_ROOT, Meca500Kinematics, pose_to_matrix
from .overlay import TargetStyle

BENCHES_DIR = REPO_ROOT / "assets" / "benches"


def _range(v) -> tuple[float, float]:
    """A scalar means a fixed value; a two-element list means a uniform range."""
    if isinstance(v, (int, float)):
        return float(v), float(v)
    lo, hi = v
    return float(lo), float(hi)


@dataclass
class BenchConfig:
    rig: str
    root: Path
    raw: dict = field(repr=False)

    # ---------------------------------------------------------------- loading

    @classmethod
    def load(cls, rig: str, benches_dir: Path = BENCHES_DIR) -> BenchConfig:
        root = Path(benches_dir) / rig
        path = root / "bench.yaml"
        if not path.exists():
            raise FileNotFoundError(
                f"No bench config for rig {rig!r} at {path}. Copy assets/benches/_template/ "
                f"to assets/benches/{rig}/ and fill it in."
            )
        return cls(rig=rig, root=root, raw=yaml.safe_load(path.read_text()) or {})

    def section(self, name: str) -> dict:
        return self.raw.get(name) or {}

    # ---------------------------------------------------------------- tool + kinematics

    def tool_transform(self) -> np.ndarray:
        """Tip frame in the flange frame: ``tool.tip_pose_in_flange`` as a Meca pose
        [x, y, z, alpha, beta, gamma]. Its z axis must point along the pipette, out of the tip."""
        pose = self.section("tool").get("tip_pose_in_flange")
        if pose is None:
            raise ValueError(f"{self.root / 'bench.yaml'}: tool.tip_pose_in_flange is required")
        return pose_to_matrix(pose)

    def kinematics(self) -> Meca500Kinematics:
        return Meca500Kinematics(
            tool=self.tool_transform(),
            joint_limit_margin_deg=float(self.section("safety").get("joint_limit_margin_deg", 2.0)),
        )

    # ---------------------------------------------------------------- obstacles

    def obstacle_fields(self) -> list:
        obs = self.section("obstacles")
        fields: list = []
        meshes = [self.root / p for p in obs.get("meshes", [])]
        missing = [str(p) for p in meshes if not p.exists()]
        if missing:
            raise FileNotFoundError(f"obstacle meshes listed in bench.yaml are missing: {missing}")
        if meshes:
            fields.append(GridField.from_meshes(meshes, pitch=float(obs.get("sdf_pitch_mm", 2.0))))
        for b in obs.get("boxes", []):
            fields.append(BoxField(b["center"], b["size"], b.get("yaw_deg", 0.0), b.get("name", "box")))
        for c in obs.get("cylinders", []):
            fields.append(CylinderField(c["base"], c["radius"], c["height"], c.get("name", "cylinder")))
        reg = self.registration_transform()
        if reg is not None:
            fields = [TransformedField(f, reg) for f in fields]
        if not fields:
            raise ValueError(
                f"{self.root / 'bench.yaml'} defines no obstacles. Refusing to plan against an empty "
                "scene - that would approve every path."
            )
        return fields

    def registration_path(self) -> Path:
        return self.root / "registration.json"

    def registration_transform(self) -> np.ndarray | None:
        """CAD -> robot correction from register_bench.py, if one was saved with apply: true.

        Off by default: if the frame is bolted where the CAD says, the touch points only
        CONFIRM the model, and silently moving obstacles by a fit to a handful of hand-jogged
        points would be worse than leaving them where they were measured.
        """
        path = self.registration_path()
        if not path.exists():
            return None
        reg = json.loads(path.read_text())
        return np.array(reg["t_robot_cad"]) if reg.get("apply") else None

    def collision_checker(self, kin: Meca500Kinematics | None = None) -> CollisionChecker:
        kin = kin or self.kinematics()
        tool = self.section("tool")
        safety = self.section("safety")
        tool_mesh = self.root / tool["mesh"] if tool.get("mesh") else None
        return CollisionChecker(
            kin=kin,
            fields=self.obstacle_fields(),
            points=RobotPoints.build(kin, tool_mesh, spacing_mm=float(safety.get("point_spacing_mm", 4.0))),
            link_margin_mm=float(safety.get("link_margin_mm", 15.0)),
            tool_margin_mm=float(safety.get("tool_margin_mm", 5.0)),
            tip_exempt_mm=float(tool.get("tip_exempt_mm", 10.0)),
        )

    # ---------------------------------------------------------------- calibration + overlay

    def calibration_path(self) -> Path:
        return self.root / self.section("scope").get("calibration", "scope_calibration.json")

    def calibration(self) -> ScopeCalibration:
        path = self.calibration_path()
        if not path.exists():
            raise FileNotFoundError(
                f"No scope calibration at {path}. Run scripts/meca500/calibrate_scope.py."
            )
        return ScopeCalibration.load(path)

    def target_style(self) -> TargetStyle:
        style = self.section("target").get("style") or {}
        if "color" in style:
            style["color"] = tuple(style["color"])
        if style.get("outline_color") is not None:
            style["outline_color"] = tuple(style["outline_color"])
        return TargetStyle(**style)

    @property
    def camera_key(self) -> str:
        return self.section("scope").get("camera_key", "microscope_cam")


@dataclass
class PlannerLimits:
    """The randomisation and speed limits, flattened out of bench.yaml with defaults."""

    fps: int = 20
    # start
    start_nominal_joints: list[float] | None = None
    start_pos_jitter_mm: tuple[float, float, float] = (20.0, 20.0, 10.0)
    start_rot_jitter_deg: float = 3.0
    start_min_height_mm: float = 5.0  # above the dish surface: out of focus at the start
    # target
    target_edge_margin_px: float = 60.0
    touch_offset_mm: float = 0.0  # +ve stops above the measured surface
    touch_nominal_joints: list[float] | None = None  # sets the approach orientation
    touch_rot_jitter_deg: float = 2.0
    # path
    gate_back_mm: tuple[float, float] = (15.0, 40.0)  # straight final approach along the pipette
    pretouch_back_mm: tuple[float, float] = (1.0, 4.0)
    pretouch_lateral_mm: float = 0.3
    n_via: tuple[int, int] = (1, 2)
    via_jitter_mm: tuple[float, float, float] = (15.0, 15.0, 10.0)
    coarse_speed_mm_s: tuple[float, float] = (8.0, 15.0)
    gate_speed_mm_s: tuple[float, float] = (2.0, 4.0)
    precision_speed_mm_s: tuple[float, float] = (0.2, 0.5)
    dwell_s: tuple[float, float] = (0.5, 1.5)
    max_joint_step_deg: float = 1.0  # per frame, coarse
    max_joint_step_precision_deg: float = 0.1  # per frame, precision
    # transit (unrecorded)
    retreat_mm: float = 30.0
    transit_speed_deg_s: float = 15.0
    safe_joints: list[float] | None = None
    # safety
    min_abs_j5_deg: float = 5.0  # wrist singularity at J5 = 0
    check_step_deg: float = 0.5
    max_attempts: int = 200
    target_attempts: int = 50  # start/path resamples per target before it counts as unreachable

    @classmethod
    def from_bench(cls, bench: BenchConfig) -> PlannerLimits:
        s, t, a, m, sf = (bench.section(k) for k in ("start", "target", "approach", "motion", "safety"))
        d = cls()

        def get(sec, key, default, conv=lambda v: v):
            return conv(sec[key]) if key in sec else default

        return cls(
            fps=get(m, "fps", d.fps, int),
            start_nominal_joints=get(s, "nominal_joints", None, list),
            start_pos_jitter_mm=get(s, "pos_jitter_mm", d.start_pos_jitter_mm, tuple),
            start_rot_jitter_deg=get(s, "rot_jitter_deg", d.start_rot_jitter_deg, float),
            start_min_height_mm=get(s, "min_height_above_surface_mm", d.start_min_height_mm, float),
            target_edge_margin_px=get(t, "edge_margin_px", d.target_edge_margin_px, float),
            touch_offset_mm=get(t, "touch_offset_mm", d.touch_offset_mm, float),
            touch_nominal_joints=get(a, "touch_nominal_joints", None, list),
            touch_rot_jitter_deg=get(a, "touch_rot_jitter_deg", d.touch_rot_jitter_deg, float),
            gate_back_mm=get(a, "gate_back_mm", d.gate_back_mm, _range),
            pretouch_back_mm=get(a, "pretouch_back_mm", d.pretouch_back_mm, _range),
            pretouch_lateral_mm=get(a, "pretouch_lateral_mm", d.pretouch_lateral_mm, float),
            n_via=get(a, "n_via", d.n_via, lambda v: tuple(int(x) for x in _range(v))),
            via_jitter_mm=get(a, "via_jitter_mm", d.via_jitter_mm, tuple),
            coarse_speed_mm_s=get(m, "coarse_speed_mm_s", d.coarse_speed_mm_s, _range),
            gate_speed_mm_s=get(m, "gate_speed_mm_s", d.gate_speed_mm_s, _range),
            precision_speed_mm_s=get(m, "precision_speed_mm_s", d.precision_speed_mm_s, _range),
            dwell_s=get(m, "dwell_s", d.dwell_s, _range),
            max_joint_step_deg=get(m, "max_joint_step_deg", d.max_joint_step_deg, float),
            max_joint_step_precision_deg=get(
                m, "max_joint_step_precision_deg", d.max_joint_step_precision_deg, float
            ),
            retreat_mm=get(m, "retreat_mm", d.retreat_mm, float),
            transit_speed_deg_s=get(m, "transit_speed_deg_s", d.transit_speed_deg_s, float),
            safe_joints=get(sf, "safe_joints", None, list),
            min_abs_j5_deg=get(sf, "min_abs_j5_deg", d.min_abs_j5_deg, float),
            check_step_deg=get(sf, "check_step_deg", d.check_step_deg, float),
            max_attempts=get(sf, "max_attempts", d.max_attempts, int),
            target_attempts=get(sf, "target_attempts", d.target_attempts, int),
        )
