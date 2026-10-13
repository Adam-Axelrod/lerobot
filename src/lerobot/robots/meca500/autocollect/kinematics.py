"""Meca500 forward/inverse kinematics from the URDF in ``mecademic_description/``.

Everything here is in the robot's own conventions so numbers can be compared directly with
``GetPose()`` / ``GetJoints()``:

- lengths in **mm**, joint angles in **degrees**;
- the world frame is the Meca's base reference frame (BRF): origin at the bottom of the base,
  z up. ``meca_base_link`` in the URDF sits there (its collision mesh spans z = 0..93 mm);
- the flange frame is the Meca's FRF. The URDF's last link frame has its x axis out of the
  flange, while the FRF has z out of the flange, so FRF = link_6 * Ry(+90 deg). At zero joints
  this gives the Meca's documented zero pose, (190, 0, 308) mm with Euler (0, 90, 0);
- Euler angles follow the Meca's mobile-XYZ convention: R = Rx(alpha) Ry(beta) Rz(gamma).

The URDF's agreement with the physical arm is an assumption until it has been checked
against ``GetPose()`` on the robot - see ``scripts/meca500/check_kinematics.py``.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET  # nosec B405 - parses the repo's own URDF, not untrusted input
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_URDF = REPO_ROOT / "mecademic_description" / "urdf" / "meca_arm_only.urdf"
MESH_DIR = REPO_ROOT / "mecademic_description" / "meshes"

M_TO_MM = 1000.0


# ----------------------------------------------------------------------------- rotations


def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation matrix about a unit ``axis``."""
    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * (kx @ kx)


def rotation_log(r: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle, radians) of a rotation matrix."""
    cos = np.clip((np.trace(r) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(cos)
    if angle < 1e-9:
        return np.zeros(3)
    if np.pi - angle < 1e-6:
        # Near 180 deg the off-diagonal formula degenerates; take the axis from R + I.
        m = (r + np.eye(3)) / 2
        axis = m[np.argmax(np.diag(m))]
        return axis / np.linalg.norm(axis) * angle
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / (2 * np.sin(angle)) * angle


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def pose_to_matrix(pose) -> np.ndarray:
    """Meca pose ``[x, y, z, alpha, beta, gamma]`` (mm, deg, mobile XYZ) -> 4x4."""
    x, y, z, a, b, g = (float(v) for v in pose)
    t = np.eye(4)
    t[:3, :3] = rot_x(np.radians(a)) @ rot_y(np.radians(b)) @ rot_z(np.radians(g))
    t[:3, 3] = (x, y, z)
    return t


def matrix_to_pose(t: np.ndarray) -> np.ndarray:
    """4x4 -> Meca pose ``[x, y, z, alpha, beta, gamma]`` (mm, deg, mobile XYZ)."""
    r = t[:3, :3]
    beta = np.arcsin(np.clip(r[0, 2], -1.0, 1.0))
    if abs(abs(r[0, 2]) - 1) > 1e-9:
        alpha = np.arctan2(-r[1, 2], r[2, 2])
        gamma = np.arctan2(-r[0, 1], r[0, 0])
    else:  # gimbal lock: alpha and gamma are coupled; put it all in alpha
        alpha = np.arctan2(r[2, 1], r[1, 1])
        gamma = 0.0
    return np.array([*t[:3, 3], *np.degrees([alpha, beta, gamma])])


def make_transform(r: np.ndarray | None = None, p=None) -> np.ndarray:
    t = np.eye(4)
    if r is not None:
        t[:3, :3] = r
    if p is not None:
        t[:3, 3] = p
    return t


# ----------------------------------------------------------------------------- URDF chain


@dataclass(frozen=True)
class Joint:
    name: str
    child: str
    origin: np.ndarray  # 4x4, parent link -> joint frame at q = 0, mm
    axis: np.ndarray  # unit vector in the joint frame
    lower_deg: float
    upper_deg: float


def _parse_origin(el) -> np.ndarray:
    if el is None:
        return np.eye(4)
    xyz = np.array([float(v) for v in el.get("xyz", "0 0 0").split()]) * M_TO_MM
    rpy = np.array([float(v) for v in el.get("rpy", "0 0 0").split()])
    return make_transform(rpy_matrix(rpy), xyz)


def load_chain(urdf_path: Path = DEFAULT_URDF) -> tuple[str, list[Joint], dict[str, Path]]:
    """Parse a serial revolute chain. Returns (base link, joints in order, link -> collision mesh)."""
    root = ET.parse(urdf_path).getroot()  # nosec B314 - trusted repo file

    meshes: dict[str, Path] = {}
    for link in root.findall("link"):
        mesh = link.find("collision/geometry/mesh")
        if mesh is not None:
            # The URDF carries absolute Windows paths from whoever generated it; resolve by
            # file name against this checkout instead so it works from any clone.
            meshes[link.get("name")] = MESH_DIR / Path(mesh.get("filename")).name

    joints_by_parent = {}
    for j in root.findall("joint"):
        if j.get("type") not in ("revolute", "continuous"):
            continue
        limit = j.find("limit")
        joint = Joint(
            name=j.get("name"),
            child=j.find("child").get("link"),
            origin=_parse_origin(j.find("origin")),
            axis=np.array([float(v) for v in j.find("axis").get("xyz").split()]),
            lower_deg=float(np.degrees(float(limit.get("lower")))),
            upper_deg=float(np.degrees(float(limit.get("upper")))),
        )
        joints_by_parent[j.find("parent").get("link")] = joint

    children = {j.child for j in joints_by_parent.values()}
    bases = [p for p in joints_by_parent if p not in children]
    if len(bases) != 1:
        raise ValueError(f"{urdf_path} is not a single serial chain (roots: {bases})")
    base = bases[0]

    chain, link = [], base
    while link in joints_by_parent:
        chain.append(joints_by_parent[link])
        link = chain[-1].child
    return base, chain, meshes


# The URDF's last link has x out of the flange; the Meca FRF has z out of the flange.
LINK6_TO_FLANGE = make_transform(rot_y(np.pi / 2))


class Meca500Kinematics:
    """FK/IK for the Meca500 plus a rigid tool (flange -> tip transform, mm).

    ``tool`` is the tip frame expressed in the flange frame (FRF). Its z axis is taken as the
    pipette's axis, pointing out of the tip - the direction the tip travels when it pushes
    "forward" into the dish.
    """

    def __init__(
        self,
        tool: np.ndarray | None = None,
        urdf_path: Path = DEFAULT_URDF,
        joint_limit_margin_deg: float = 2.0,
    ):
        self.base_link, self.joints, self.link_meshes = load_chain(urdf_path)
        if len(self.joints) != 6:
            raise ValueError(f"expected 6 joints in {urdf_path}, found {len(self.joints)}")
        self.tool = np.eye(4) if tool is None else np.asarray(tool, dtype=float)
        self.lower = np.array([j.lower_deg for j in self.joints]) + joint_limit_margin_deg
        self.upper = np.array([j.upper_deg for j in self.joints]) - joint_limit_margin_deg

    # ------------------------------------------------------------------ forward

    def link_frames(self, q_deg) -> np.ndarray:
        """World poses of [base, link_1 .. link_6] as a (7, 4, 4) array."""
        q = np.radians(np.asarray(q_deg, dtype=float))
        frames = np.empty((7, 4, 4))
        t = np.eye(4)
        frames[0] = t
        for i, joint in enumerate(self.joints):
            t = t @ joint.origin @ make_transform(axis_angle(joint.axis, q[i]))
            frames[i + 1] = t
        return frames

    def flange(self, q_deg) -> np.ndarray:
        return self.link_frames(q_deg)[-1] @ LINK6_TO_FLANGE

    def tip(self, q_deg) -> np.ndarray:
        return self.flange(q_deg) @ self.tool

    def jacobian(self, q_deg) -> np.ndarray:
        """Geometric Jacobian of the tip, (6, 6): rows [v (mm/rad); w (rad/rad)]."""
        frames = self.link_frames(q_deg)
        p_tip = (frames[-1] @ LINK6_TO_FLANGE @ self.tool)[:3, 3]
        jac = np.empty((6, 6))
        for i, joint in enumerate(self.joints):
            # Joint i rotates link i+1; its axis and origin live in frame (parent @ origin).
            t_joint = frames[i] @ joint.origin
            z = t_joint[:3, :3] @ (joint.axis / np.linalg.norm(joint.axis))
            jac[:3, i] = np.cross(z, p_tip - t_joint[:3, 3])
            jac[3:, i] = z
        return jac

    def within_limits(self, q_deg) -> bool:
        q = np.asarray(q_deg)
        return bool(np.all(q >= self.lower) and np.all(q <= self.upper))

    # ------------------------------------------------------------------ inverse

    def ik(
        self,
        target: np.ndarray,
        seed_deg,
        pos_tol_mm: float = 0.001,
        rot_tol_deg: float = 0.001,
        max_iters: int = 200,
        rot_weight_mm: float = 100.0,
    ) -> np.ndarray | None:
        """Damped-least-squares IK for the tip pose ``target`` (4x4), from ``seed_deg``.

        Seeding from the neighbouring waypoint keeps a path in one arm configuration, so it
        never flips elbow/wrist mid-trajectory. Returns degrees, or None if it did not
        converge inside the joint limits. ``rot_weight_mm`` makes 1 rad of orientation error
        cost as much as that many mm of position error.
        """
        q = np.radians(np.asarray(seed_deg, dtype=float))
        lo, hi = np.radians(self.lower), np.radians(self.upper)
        weights = np.array([1, 1, 1, rot_weight_mm, rot_weight_mm, rot_weight_mm])
        damping = 1e-2
        for _ in range(max_iters):
            current = self.tip(np.degrees(q))
            dp = target[:3, 3] - current[:3, 3]
            dr = rotation_log(target[:3, :3] @ current[:3, :3].T)
            if np.linalg.norm(dp) < pos_tol_mm and np.degrees(np.linalg.norm(dr)) < rot_tol_deg:
                q_deg = np.degrees(q)
                return q_deg if self.within_limits(q_deg) else None
            err = np.concatenate([dp, dr]) * weights
            jac = self.jacobian(np.degrees(q)) * weights[:, None]
            jjt = jac @ jac.T
            step = jac.T @ np.linalg.solve(jjt + (damping**2) * np.eye(6), err)
            # Cap the step so a far seed walks rather than leaps across configurations.
            step *= min(1.0, np.radians(10) / max(np.abs(step).max(), 1e-12))
            q = np.clip(q + step, lo, hi)
        return None

    # ------------------------------------------------------------------ helpers

    def manipulability(self, q_deg) -> float:
        """Yoshikawa manipulability of the position Jacobian (mm^3); ~0 near singularities."""
        jv = self.jacobian(q_deg)[:3]
        return float(np.sqrt(max(np.linalg.det(jv @ jv.T), 0.0)))


# ----------------------------------------------------------------------------- meshes


def load_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read an STL (binary or ASCII) into (vertices (N,3), faces (M,3)) without dependencies."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        n = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
        if 84 + 50 * n == len(data):
            rec = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            tris = np.frombuffer(data, dtype=rec, count=n, offset=84)["v"].astype(float)
            return tris.reshape(-1, 3), np.arange(3 * n).reshape(-1, 3)
    verts = [
        [float(x) for x in line.split()[1:4]]
        for line in data.decode("ascii", errors="ignore").splitlines()
        if line.strip().startswith("vertex")
    ]
    tris = np.array(verts, dtype=float)
    return tris, np.arange(len(tris)).reshape(-1, 3)


def sample_surface(
    vertices: np.ndarray, faces: np.ndarray, spacing: float, rng: np.random.Generator
) -> np.ndarray:
    """Points on a triangle mesh, roughly one per ``spacing``^2 of area, plus every vertex."""
    tri = vertices[faces]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    n = max(int(area.sum() / spacing**2), 1)
    idx = rng.choice(len(tri), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = tri[idx]
    pts = t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])
    return np.vstack([pts, np.unique(vertices, axis=0)])
