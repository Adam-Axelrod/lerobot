"""Clearance between the Meca500 (+ tool) and the bench's obstacles.

The obstacle side is a union of *fields*, each answering "signed distance from these points
to me" (negative = inside):

- ``BoxField`` / ``CylinderField``: analytic primitives. Good for a quick hand-measured model
  before the CAD exists, for things that move (the dish), and for tests;
- ``GridField``: a signed-distance grid built once from STL meshes exported from Fusion 360
  (needs the ``meca500-autocollect`` extra: trimesh + scipy) and cached next to the STLs.

The robot side is a point cloud sampled on each link's collision mesh (from the URDF) and on
the tool mesh, carried by forward kinematics. A configuration is clear when every robot point
is at least its margin away from every obstacle. Points on the last few mm of the pipette
are exempt, because the tip has to touch the dish by design.

The grid is conservative: lookups subtract half a voxel diagonal, so a reported clearance is
never larger than the true one by more than rounding.
"""

from __future__ import annotations

import hashlib
import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .kinematics import LINK6_TO_FLANGE, Meca500Kinematics, load_stl, sample_surface

logger = logging.getLogger(__name__)

# ----------------------------------------------------------------------------- obstacle fields


class BoxField:
    """Axis-aligned box (optionally yawed about z), in the base frame, mm."""

    def __init__(self, center, size, yaw_deg: float = 0.0, name: str = "box"):
        self.name = name
        self.center = np.asarray(center, dtype=float)
        self.half = np.asarray(size, dtype=float) / 2
        c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        self.r_world_box = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def distance(self, pts: np.ndarray) -> np.ndarray:
        local = (pts - self.center) @ self.r_world_box  # rotate into box axes
        q = np.abs(local) - self.half
        outside = np.linalg.norm(np.maximum(q, 0), axis=1)
        inside = np.minimum(q.max(axis=1), 0)
        return outside + inside


class CylinderField:
    """Vertical cylinder: ``base`` is the centre of its bottom face, mm."""

    def __init__(self, base, radius: float, height: float, name: str = "cylinder"):
        self.name = name
        self.base = np.asarray(base, dtype=float)
        self.radius = float(radius)
        self.height = float(height)

    def distance(self, pts: np.ndarray) -> np.ndarray:
        d = pts - self.base
        dr = np.linalg.norm(d[:, :2], axis=1) - self.radius
        dz = np.abs(d[:, 2] - self.height / 2) - self.height / 2
        q = np.stack([dr, dz], axis=1)
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


class GridField:
    """Signed-distance grid sampled from closed meshes. See ``from_meshes``."""

    def __init__(self, sdf: np.ndarray, origin: np.ndarray, pitch: float, name: str = "meshes"):
        self.name = name
        self.sdf = sdf
        self.origin = np.asarray(origin, dtype=float)
        self.pitch = float(pitch)
        self.shape = np.array(sdf.shape)
        self._slack = pitch * np.sqrt(3) / 2  # worst-case voxel rounding

    @classmethod
    def from_meshes(
        cls, paths: list[Path], pitch: float = 2.0, pad: float = 60.0, cache_dir: Path | None = None
    ) -> GridField:
        """Voxelise the STLs at ``pitch`` mm and distance-transform them, caching the result.

        The cache key is a hash of the STL bytes and the parameters, so re-exporting from
        Fusion invalidates it automatically.
        """
        paths = [Path(p) for p in paths]
        digest = hashlib.sha256(f"{pitch}:{pad}".encode())
        for p in sorted(paths):
            digest.update(p.read_bytes())
        key = digest.hexdigest()[:16]
        cache = (cache_dir or paths[0].parent) / f".sdf_cache_{key}.npz"
        if cache.exists():
            data = np.load(cache)
            return cls(data["sdf"], data["origin"], float(data["pitch"]), name="meshes")

        import trimesh
        from scipy.ndimage import distance_transform_edt

        logger.info(f"Building signed-distance grid from {len(paths)} mesh(es) at {pitch} mm ...")
        meshes = [trimesh.load_mesh(p) for p in paths]
        lo = np.min([m.bounds[0] for m in meshes], axis=0) - pad
        hi = np.max([m.bounds[1] for m in meshes], axis=0) + pad
        shape = np.ceil((hi - lo) / pitch).astype(int) + 1
        occupied = np.zeros(shape, dtype=bool)
        for m in meshes:
            if not m.is_watertight:
                logger.warning(
                    f"{getattr(m, 'metadata', {}).get('file_name', m)} is not watertight; only its "
                    "surface is treated as solid. Export closed bodies from Fusion where possible."
                )
            vox = m.voxelized(pitch).fill()
            idx = np.round((vox.points - lo) / pitch).astype(int)
            idx = idx[np.all((idx >= 0) & (idx < shape), axis=1)]
            occupied[tuple(idx.T)] = True
        outside = distance_transform_edt(~occupied) * pitch
        inside = distance_transform_edt(occupied) * pitch
        sdf = (outside - inside).astype(np.float32)
        np.savez_compressed(cache, sdf=sdf, origin=lo, pitch=pitch)
        return cls(sdf, lo, pitch)

    def distance(self, pts: np.ndarray) -> np.ndarray:
        f = (pts - self.origin) / self.pitch
        idx = np.round(f).astype(int)
        inside = np.all((idx >= 0) & (idx < self.shape), axis=1)
        d = np.empty(len(pts))
        d[inside] = self.sdf[tuple(idx[inside].T)] - self._slack
        # Beyond the padded grid every mesh is at least the padding away; report the distance
        # to the grid box plus the boundary value, which is a lower bound on the true one.
        if (~inside).any():
            lo, hi = self.origin, self.origin + (self.shape - 1) * self.pitch
            gap = np.linalg.norm(np.maximum(lo - pts[~inside], 0) + np.maximum(pts[~inside] - hi, 0), axis=1)
            d[~inside] = gap + self._border_min()
        return d

    def _border_min(self) -> float:
        if not hasattr(self, "_border"):
            s = self.sdf
            faces = [s[0], s[-1], s[:, 0], s[:, -1], s[:, :, 0], s[:, :, -1]]
            self._border = float(min(f.min() for f in faces)) - self._slack
        return self._border


class TransformedField:
    """Another field moved rigidly by ``t_world_field`` (4x4): the registration correction
    from register_bench.py, applied to obstacles modelled in the CAD frame."""

    def __init__(self, inner, t_world_field: np.ndarray):
        self.inner = inner
        self.name = inner.name
        inv = np.linalg.inv(np.asarray(t_world_field, dtype=float))
        self._r, self._p = inv[:3, :3], inv[:3, 3]

    def distance(self, pts: np.ndarray) -> np.ndarray:
        return self.inner.distance(pts @ self._r.T + self._p)


def fit_rigid(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Least-squares rigid transform (Kabsch) mapping ``src`` points onto ``dst``, as 4x4."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    u, _, vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1, 1, d]) @ u.T
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = r, cd - r @ cs
    return t


# ----------------------------------------------------------------------------- robot points


@dataclass
class RobotPoints:
    """Sample points per link, in each link's own frame (mm)."""

    link_points: list[np.ndarray]  # index i -> link frame i of Meca500Kinematics.link_frames
    tool_points: np.ndarray | None  # in the TIP frame
    tool_tip_distance: np.ndarray | None = None  # distance of each tool point from the tip

    @classmethod
    def build(
        cls,
        kin: Meca500Kinematics,
        tool_mesh: Path | None = None,
        spacing_mm: float = 4.0,
        skip_base: bool = True,
        seed: int = 0,
    ) -> RobotPoints:
        """Sample the URDF collision meshes and the tool mesh.

        The base is skipped by default: it is bolted to the frame, so it is in contact with
        the mounting plate by construction and moving the arm cannot change its clearance.
        ``tool_mesh`` is expected in the FLANGE frame (FRF), mm - the frame Fusion's tool
        body should be exported in.
        """
        rng = np.random.default_rng(seed)
        link_names = [kin.base_link] + [j.child for j in kin.joints]
        link_points = []
        for i, name in enumerate(link_names):
            mesh = kin.link_meshes.get(name)
            if mesh is None or (skip_base and i == 0):
                link_points.append(np.empty((0, 3)))
                continue
            v, f = load_stl(mesh)
            link_points.append(sample_surface(v * 1000.0, f, spacing_mm, rng))

        tool_points = tool_dist = None
        if tool_mesh is not None:
            v, f = load_stl(tool_mesh)
            pts_flange = sample_surface(v, f, min(spacing_mm, 2.0), rng)
            tip_inv = np.linalg.inv(kin.tool)
            tool_points = pts_flange @ tip_inv[:3, :3].T + tip_inv[:3, 3]
            tool_dist = np.linalg.norm(tool_points, axis=1)
        return cls(link_points, tool_points, tool_dist)


# ----------------------------------------------------------------------------- checker


@dataclass
class Clearance:
    min_mm: float
    where: str  # which robot part was closest
    obstacle: str


@dataclass
class CollisionChecker:
    kin: Meca500Kinematics
    fields: list
    points: RobotPoints
    link_margin_mm: float = 15.0
    tool_margin_mm: float = 5.0
    tip_exempt_mm: float = 10.0
    _names: list[str] = field(default_factory=list, init=False)

    def __post_init__(self):
        self._names = ["base"] + [f"link_{i}" for i in range(1, 7)]

    def _world_points(self, q_deg) -> list[tuple[str, np.ndarray, float]]:
        frames = self.kin.link_frames(q_deg)
        groups = []
        for i, pts in enumerate(self.points.link_points):
            if len(pts):
                t = frames[i]
                groups.append((self._names[i], pts @ t[:3, :3].T + t[:3, 3], self.link_margin_mm))
        if self.points.tool_points is not None:
            keep = self.points.tool_tip_distance > self.tip_exempt_mm
            t = frames[-1] @ LINK6_TO_FLANGE @ self.kin.tool
            pts = self.points.tool_points[keep]
            groups.append(("tool", pts @ t[:3, :3].T + t[:3, 3], self.tool_margin_mm))
        return groups

    def clearance(self, q_deg) -> Clearance:
        """Smallest (distance - margin) over all robot parts and obstacles; >= 0 means clear."""
        best = Clearance(np.inf, "", "")
        for part, pts, margin in self._world_points(q_deg):
            for f in self.fields:
                d = float(f.distance(pts).min()) - margin
                if d < best.min_mm:
                    best = Clearance(d, part, f.name)
        return best

    def is_clear(self, q_deg) -> bool:
        return self.clearance(q_deg).min_mm >= 0

    def check_path(self, qs: np.ndarray, max_step_deg: float = 0.5) -> Clearance:
        """Check a joint path, densified so no joint moves more than ``max_step_deg`` between
        checks. The controller interpolates linearly in joint space between MoveJoints
        targets, so densifying in joint space checks the motion the arm actually makes.
        Returns the worst clearance; stops early at the first violation.
        """
        qs = np.asarray(qs, dtype=float)
        worst = self.clearance(qs[0])
        if worst.min_mm < 0:
            return worst
        for a, b in zip(qs[:-1], qs[1:], strict=True):
            n = max(int(np.ceil(np.abs(b - a).max() / max_step_deg)), 1)
            for s in range(1, n + 1):
                c = self.clearance(a + (b - a) * s / n)
                if c.min_mm < worst.min_mm:
                    worst = c
                    if c.min_mm < 0:
                        return worst
        return worst
