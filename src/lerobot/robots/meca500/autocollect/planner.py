"""Sampling and checking one automated demonstration.

An episode is four phases, each sampled at the recording fps so frame k of the plan is the
action recorded at frame k:

    coarse     start -> random via-point(s) -> gate      smooth curve, randomised speed
    gate       gate -> pre-touch                         straight, along the pipette axis
    precision  pre-touch -> touch                        straight, slow, precision.state = 1
    dwell      hold at the touch pose                    precision.state = 1

The gate is a point a few cm back along the pipette's own axis from the target, so the last
stretch under the lamp is always a straight push along the pipette - the corridor where the
clearance is tightest is the least random part of the path, by design.

Everything is planned and checked before the arm moves. Any failure (IK, joint limits, the
wrist singularity, too large a per-frame joint step, a collision) throws the sample away and
draws another; ``PlanningError`` is raised only when ``max_attempts`` all fail, which means
the limits in bench.yaml are infeasible rather than unlucky.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

from .bench import PlannerLimits
from .calibration import ScopeCalibration
from .collision import Clearance, CollisionChecker
from .kinematics import Meca500Kinematics, axis_angle, make_transform, rotation_log

logger = logging.getLogger(__name__)


class PlanningError(RuntimeError):
    pass


class _RejectedSampleError(Exception):
    """One sample failed a check; the reason is tallied, then a new sample is drawn."""


# ----------------------------------------------------------------------------- geometry helpers


def min_jerk(n: int) -> np.ndarray:
    """``n`` samples of the min-jerk profile on (0, 1] - starts and ends at rest."""
    t = np.arange(1, n + 1) / n
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def catmull_rom(points: np.ndarray, samples_per_segment: int = 60) -> np.ndarray:
    """Centripetal Catmull-Rom polyline through ``points`` (k, 3); no overshoot loops."""
    p = np.vstack([2 * points[0] - points[1], points, 2 * points[-1] - points[-2]])
    out = [points[0]]
    for i in range(1, len(p) - 2):
        p0, p1, p2, p3 = p[i - 1], p[i], p[i + 1], p[i + 2]
        t0 = 0.0
        t1 = t0 + max(np.linalg.norm(p1 - p0), 1e-6) ** 0.5
        t2 = t1 + max(np.linalg.norm(p2 - p1), 1e-6) ** 0.5
        t3 = t2 + max(np.linalg.norm(p3 - p2), 1e-6) ** 0.5
        for t in np.linspace(t1, t2, samples_per_segment + 1)[1:]:
            a1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1
            a2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2
            a3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
            b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3
            out.append((t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2)
    return np.array(out)


def resample_by_arclength(poly: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Points at arc-length fractions ``s`` in [0, 1] along a polyline."""
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    target = s * cum[-1]
    return np.stack([np.interp(target, cum, poly[:, k]) for k in range(3)], axis=1)


def polyline_length(poly: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(poly, axis=0), axis=1).sum())


def slerp(r0: np.ndarray, r1: np.ndarray, s: float) -> np.ndarray:
    w = rotation_log(r1 @ r0.T)
    angle = np.linalg.norm(w)
    if angle < 1e-12:
        return r0
    return axis_angle(w / angle, angle * s) @ r0


def random_rotation(rng: np.random.Generator, max_deg: float) -> np.ndarray:
    if max_deg <= 0:
        return np.eye(3)
    axis = rng.normal(size=3)
    return axis_angle(axis / np.linalg.norm(axis), np.radians(rng.uniform(0, max_deg)))


# ----------------------------------------------------------------------------- plans


@dataclass
class EpisodePlan:
    joints: np.ndarray  # (N, 6) deg, one row per recorded frame
    precision: np.ndarray  # (N,) 0.0 / 1.0
    fps: int
    seed: int
    target_px: tuple[float, float]
    target_xyz: tuple[float, float, float]
    clearance: Clearance
    phase_frames: dict[str, int] = field(default_factory=dict)

    @property
    def start_joints(self) -> np.ndarray:
        return self.joints[0]

    @property
    def duration_s(self) -> float:
        return len(self.joints) / self.fps

    def meta(self) -> dict:
        """JSON-safe record for episodes_meta.jsonl (deliberately NOT dataset features)."""
        return {
            "seed": int(self.seed),
            "target_px": [round(float(v), 2) for v in self.target_px],
            "target_xyz_mm": [round(float(v), 4) for v in self.target_xyz],
            "start_joints_deg": [round(float(v), 4) for v in self.start_joints],
            "n_frames": len(self.joints),
            "phase_frames": self.phase_frames,
            "min_clearance_mm": round(float(self.clearance.min_mm), 2),
            "closest_part": self.clearance.where,
            "closest_obstacle": self.clearance.obstacle,
        }


class Planner:
    def __init__(
        self,
        kin: Meca500Kinematics,
        checker: CollisionChecker,
        calib: ScopeCalibration,
        limits: PlannerLimits,
    ):
        self.kin, self.checker, self.calib, self.lim = kin, checker, calib, limits
        if limits.touch_nominal_joints is None:
            raise ValueError(
                "approach.touch_nominal_joints is not set in bench.yaml. It fixes the pipette's "
                "approach orientation; take it from the final frame of a good human demo "
                "(scripts/meca500/check_demos_clearance.py prints candidates)."
            )
        if limits.start_nominal_joints is None:
            raise ValueError("start.nominal_joints is not set in bench.yaml (see check_demos_clearance.py).")
        self.touch_nominal = self.kin.tip(limits.touch_nominal_joints)
        self.start_nominal = self.kin.tip(limits.start_nominal_joints)
        self.rejections: dict[str, int] = {}

    # ------------------------------------------------------------------ episode

    def sample_episode(self, seed: int) -> EpisodePlan:
        rng = np.random.default_rng(seed)
        self.rejections = {}
        # The target is drawn ONCE and kept while start/path are resampled. Redrawing it on
        # every rejection would make hard-to-reach parts of the image proportionally rarer in
        # the dataset (on Huanyu's setup acceptance ranges 22-59% across the image), so the
        # policy would see a skewed target distribution. Only a target that fails
        # `target_attempts` times in a row is replaced - i.e. one that is genuinely unreachable.
        target_px = self.sample_target_px(rng)
        for attempt in range(self.lim.max_attempts):
            if attempt and attempt % self.lim.target_attempts == 0:
                self.rejections["target replaced (unreachable)"] = (
                    self.rejections.get("target replaced (unreachable)", 0) + 1
                )
                target_px = self.sample_target_px(rng)
            try:
                plan = self._try_episode(rng, target_px)
                plan.seed = seed
                return plan
            except _RejectedSampleError as r:
                self.rejections[str(r)] = self.rejections.get(str(r), 0) + 1
        raise PlanningError(
            f"No feasible episode in {self.lim.max_attempts} attempts (seed {seed}). "
            f"Rejections: {self.rejections}. The limits in bench.yaml are infeasible together."
        )

    def sample_target_px(self, rng: np.random.Generator) -> np.ndarray:
        w, h = self.calib.image_size
        m = self.lim.target_edge_margin_px
        return np.array([rng.uniform(m, w - 1 - m), rng.uniform(m, h - 1 - m)])

    def _try_episode(self, rng: np.random.Generator, target_px=None) -> EpisodePlan:
        lim = self.lim
        surface = self.calib.surface_z_mm

        # --- target and approach geometry
        if target_px is None:
            target_px = self.sample_target_px(rng)
        target_px = np.asarray(target_px, dtype=float)
        xy = self.calib.pixel_to_xy(target_px)[0]
        p_touch = np.array([xy[0], xy[1], surface + lim.touch_offset_mm])
        r_touch = random_rotation(rng, lim.touch_rot_jitter_deg) @ self.touch_nominal[:3, :3]
        axis = r_touch[:, 2]  # pipette axis, pointing out of the tip
        lateral = rng.normal(size=3)
        lateral -= axis * (lateral @ axis)
        lateral *= rng.uniform(0, lim.pretouch_lateral_mm) / max(np.linalg.norm(lateral), 1e-9)
        p_pre = p_touch - axis * rng.uniform(*lim.pretouch_back_mm) + lateral
        p_gate = p_touch - axis * rng.uniform(*lim.gate_back_mm)

        # --- start
        p_start = self.start_nominal[:3, 3] + rng.uniform(-1, 1, 3) * np.asarray(lim.start_pos_jitter_mm)
        if p_start[2] < surface + lim.start_min_height_mm:
            raise _RejectedSampleError("start too low")
        r_start = random_rotation(rng, lim.start_rot_jitter_deg) @ self.start_nominal[:3, :3]
        q_start = self.kin.ik(make_transform(r_start, p_start), lim.start_nominal_joints)
        if q_start is None:
            raise _RejectedSampleError("start IK")

        # --- coarse curve: start -> via(s) -> gate
        n_via = int(rng.integers(lim.n_via[0], lim.n_via[1] + 1))
        fracs = np.sort(rng.uniform(0.25, 0.75, n_via))
        vias = [
            p_start + f * (p_gate - p_start) + rng.uniform(-1, 1, 3) * np.asarray(lim.via_jitter_mm)
            for f in fracs
        ]
        coarse_poly = catmull_rom(np.vstack([p_start, *vias, p_gate]))

        phases = [
            ("coarse", coarse_poly, r_start, r_touch, lim.coarse_speed_mm_s, lim.max_joint_step_deg, 0.0),
            (
                "gate",
                np.vstack([p_gate, p_pre]),
                r_touch,
                r_touch,
                lim.gate_speed_mm_s,
                lim.max_joint_step_deg,
                0.0,
            ),
            (
                "precision",
                np.vstack([p_pre, p_touch]),
                r_touch,
                r_touch,
                lim.precision_speed_mm_s,
                lim.max_joint_step_precision_deg,
                1.0,
            ),
        ]

        joints, precision, phase_frames = [q_start], [0.0], {}
        for name, poly, r0, r1, speed, max_step, flag in phases:
            qs = self._time_and_solve(poly, r0, r1, rng.uniform(*speed), max_step, joints[-1], name)
            joints.extend(qs)
            precision.extend([flag] * len(qs))
            phase_frames[name] = len(qs)

        n_dwell = max(int(round(rng.uniform(*lim.dwell_s) * lim.fps)), 1)
        joints.extend([joints[-1]] * n_dwell)
        precision.extend([1.0] * n_dwell)
        phase_frames["dwell"] = n_dwell
        joints = np.array(joints)

        if np.min(np.abs(joints[:, 4])) < lim.min_abs_j5_deg:
            raise _RejectedSampleError("wrist singularity")
        clearance = self.checker.check_path(joints, lim.check_step_deg)
        if clearance.min_mm < 0:
            raise _RejectedSampleError(f"collision ({clearance.where} vs {clearance.obstacle})")

        return EpisodePlan(
            joints=joints,
            precision=np.array(precision),
            fps=lim.fps,
            seed=-1,
            target_px=(float(target_px[0]), float(target_px[1])),
            target_xyz=tuple(float(v) for v in p_touch),
            clearance=clearance,
            phase_frames=phase_frames,
        )

    def _time_and_solve(self, poly, r0, r1, speed, max_step, q_seed, name) -> list[np.ndarray]:
        """Sample a Cartesian phase at fps with a min-jerk profile and solve IK frame by frame.

        If any frame-to-frame joint step exceeds ``max_step`` the phase is re-timed slower
        (never faster than the sampled speed), so the joint-space rate always stays inside
        what the human demos showed.
        """
        length = polyline_length(poly)
        duration = max(length / speed, 1.0 / self.lim.fps)
        for _ in range(6):
            n = max(int(np.ceil(duration * self.lim.fps)), 1)
            s = min_jerk(n)
            pts = resample_by_arclength(poly, s)
            qs, seed = [], q_seed
            for k in range(n):
                q = self.kin.ik(make_transform(slerp(r0, r1, s[k]), pts[k]), seed)
                if q is None:
                    raise _RejectedSampleError(f"{name} IK")
                qs.append(q)
                seed = q
            steps = np.abs(np.diff(np.vstack([q_seed, *qs]), axis=0)).max()
            if steps <= max_step:
                return qs
            duration *= steps / max_step * 1.05
        raise _RejectedSampleError(f"{name} joint speed")

    # ------------------------------------------------------------------ transit

    def plan_transit(self, q_from, q_to, retreat: bool = True) -> np.ndarray:
        """Unrecorded move between episodes: back off along the pipette, then joint-space.

        Joint-space moves are tried direct first, then through ``safety.safe_joints``. Every
        candidate goes through the same collision check as a recorded episode.
        """
        q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
        path = [q_from]
        if retreat and self.lim.retreat_mm > 0:
            t = self.kin.tip(q_from)
            p_back = t[:3, 3] - t[:3, 2] * self.lim.retreat_mm
            try:
                path.extend(
                    self._time_and_solve(
                        np.vstack([t[:3, 3], p_back]),
                        t[:3, :3],
                        t[:3, :3],
                        5.0,
                        self.lim.max_joint_step_deg,
                        q_from,
                        "retreat",
                    )
                )
            except _RejectedSampleError:
                logger.warning("Retreat along the pipette axis failed IK; skipping it.")
        q_mid = path[-1]

        candidates = [[q_mid, q_to]]
        if self.lim.safe_joints is not None:
            candidates.append([q_mid, np.asarray(self.lim.safe_joints, float), q_to])
        last = None
        for via in candidates:
            legs = [self._joint_move(a, b) for a, b in zip(via[:-1], via[1:], strict=True)]
            qs = np.vstack([np.vstack(path), *legs])
            last = self.checker.check_path(qs, self.lim.check_step_deg)
            if last.min_mm >= 0:
                return qs
        raise PlanningError(
            f"No collision-free transit found (closest: {last.where} vs {last.obstacle}, "
            f"{last.min_mm:.1f} mm inside the margin). Set or fix safety.safe_joints in bench.yaml."
        )

    def _joint_move(self, a, b) -> np.ndarray:
        span = np.abs(b - a).max()
        n = max(int(np.ceil(span / self.lim.transit_speed_deg_s * self.lim.fps * 1.875)), 1)
        # 1.875 = peak/mean speed ratio of min-jerk, so the PEAK stays at transit_speed.
        return a + np.outer(min_jerk(n), b - a)
