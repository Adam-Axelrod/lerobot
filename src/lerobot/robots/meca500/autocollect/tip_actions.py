"""Tip-motion actions in the camera's axes: the action space of image-only policies.

A joint-delta policy has to work out the arm's kinematics from images, and with the joint
state as input it reads the target from the joints instead (the 2026-11-05 shortcut). Here
each action is the tip's own motion, expressed along the image axes:

    [du, dv, dz, ru, rv, rz]

``du, dv`` (mm) are the tip's motion in the dish plane along the image's u (right) and v (down)
axes, ``dz`` (mm) is along base Z, and ``ru, rv, rz`` (rad) are the orientation change as a
rotation vector in the same axes. The mapping from base X/Y to image axes is the orthogonal
part of the scope calibration, so the scale stays in mm and a mirrored camera is fine.

In these axes the right action does not depend on where things are in the frame: shift the
image, and the tip and the target move together while "0.3 mm right" stays right. That is
what makes image-shift augmentation label-preserving here, where it was not for joint deltas.

Forward kinematics turns recorded joint targets into these actions; at deployment, inverse
kinematics turns them back into joint targets (seeded at the current goal, so the arm stays in
one configuration).
"""

import numpy as np

from .calibration import ScopeCalibration
from .kinematics import Meca500Kinematics

ACTION_NAMES = ["tip_u.mm", "tip_v.mm", "tip_z.mm", "rot_u.rad", "rot_v.rad", "rot_z.rad", "precision.state"]


def camera_axes(calib: ScopeCalibration) -> np.ndarray:
    """2x2 orthogonal map from base-frame X/Y (mm) to image u/v axes (mm)."""
    xy_to_px = np.linalg.inv(np.asarray(calib.px_to_xy, dtype=float)[:, :2])
    u, _, vt = np.linalg.svd(xy_to_px)
    return u @ vt


def _lift(r2: np.ndarray) -> np.ndarray:
    """The 2x2 X/Y map as a 3x3 map that leaves Z alone."""
    m = np.eye(3)
    m[:2, :2] = r2
    return m


def rotvec(r: np.ndarray) -> np.ndarray:
    angle = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
    if angle < 1e-12:
        return np.zeros(3)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w * angle / (2 * np.sin(angle))


def rotmat(v: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * kx + (1 - np.cos(angle)) * kx @ kx


def tip_delta(t_from: np.ndarray, t_to: np.ndarray, axes: np.ndarray) -> np.ndarray:
    """The action that moves tip pose ``t_from`` (4x4) to ``t_to``, in camera axes."""
    m = _lift(axes)
    dp = m @ (t_to[:3, 3] - t_from[:3, 3])
    dr = m @ rotvec(t_to[:3, :3] @ t_from[:3, :3].T)
    return np.concatenate([dp, dr])


def apply_tip_delta(t_from: np.ndarray, action: np.ndarray, axes: np.ndarray) -> np.ndarray:
    """Inverse of ``tip_delta``: the tip pose ``action`` (6,) leads to from ``t_from``."""
    m_inv = _lift(axes).T
    out = np.eye(4)
    out[:3, 3] = t_from[:3, 3] + m_inv @ action[:3]
    out[:3, :3] = rotmat(m_inv @ action[3:6]) @ t_from[:3, :3]
    return out


def joints_to_tip_actions(kin: Meca500Kinematics, joints_deg: np.ndarray, axes: np.ndarray) -> np.ndarray:
    """Forward increments between successive joint targets (N, 6) as tip actions (N, 6).

    As in convert_to_delta_actions.py: element k moves target k to target k+1, and the last
    frame holds (all zeros)."""
    poses = [kin.tip(q) for q in joints_deg]
    out = np.zeros((len(poses), 6))
    for k in range(len(poses) - 1):
        out[k] = tip_delta(poses[k], poses[k + 1], axes)
    return out


def tip_action_to_joints(
    kin: Meca500Kinematics, t_goal: np.ndarray, q_goal_deg: np.ndarray, action: np.ndarray, axes: np.ndarray
) -> tuple[np.ndarray, np.ndarray | None]:
    """Carry out ``action`` from the goal tip pose ``t_goal`` (reached by ``q_goal_deg``).

    Returns the new goal pose and its joint target (None if IK fails). Integrate onto the goal
    POSE, not onto FK of the last IK solution: each solve is only good to its tolerance, and
    re-reading the pose from the joints adds those errors up over an episode (27 um at 600 steps)."""
    t_next = apply_tip_delta(t_goal, action, axes)
    return t_next, kin.ik(t_next, q_goal_deg)
