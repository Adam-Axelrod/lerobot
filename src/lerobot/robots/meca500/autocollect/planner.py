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


def sample_target_px(rng: np.random.Generator, lim: PlannerLimits, image_size) -> np.ndarray:
    """A uniform target pixel, ``target_edge_margin_px`` inside the image and, if the lit field is
    set, within ``target_field_radius_frac`` of it. In the dark corners the tip detector misses
    the tip: on 2026-11-04 the two targets beyond the lit circle gave the one unmeasured episode
    and the one 79 um miss, while all six inside it ended within 50 um."""
    w, h = image_size
    m = lim.target_edge_margin_px
    for _ in range(1000):
        px = np.array([rng.uniform(m, w - 1 - m), rng.uniform(m, h - 1 - m)])
        if target_px_allowed(px, lim, image_size):
            return px
    raise ValueError("No target pixel inside both the edge margin and the lit field: check bench.yaml target.")


def target_px_allowed(px, lim: PlannerLimits, image_size) -> bool:
    """Inside the edge margin and, if the lit field is set, within its allowed fraction."""
    w, h = image_size
    m = lim.target_edge_margin_px
    if not (m <= px[0] <= w - 1 - m and m <= px[1] <= h - 1 - m):
        return False
    if lim.target_field_centre_px is None or lim.target_field_radius_px is None:
        return True
    return np.linalg.norm(np.asarray(px) - lim.target_field_centre_px) <= lim.target_field_radius_frac * lim.target_field_radius_px


def sample_decoy_px(
    rng: np.random.Generator, true_px, max_mm: float, lim: PlannerLimits, calib, min_mm: float = 0.0
) -> np.ndarray:
    """A pixel up to ``max_mm`` (uniform in distance, uniform direction, in the dish plane) from
    ``true_px``, inside the allowed target region.

    The recorder plans the whole path to this DECOY while the marker sits on the true target, and
    the vision correction then closes the gap. Without it the pre-touch pose was the target plus a
    planned offset, so the joint state told the policy where to go and the first ACT model used
    the marker to set off but not to finish (2026-11-05 probes: 1.4% response to a moved marker
    at the pre-touch point, 7% near the dish).
    """
    true_px = np.asarray(true_px, float)
    if max_mm <= 0:
        return true_px
    xy0 = calib.pixel_to_xy(true_px)[0]
    for _ in range(1000):
        ang = rng.uniform(0, 2 * np.pi)
        d = rng.uniform(min_mm, max_mm) * np.array([np.cos(ang), np.sin(ang)])
        px = calib.xy_to_pixel(xy0 + d)[0]
        if target_px_allowed(px, lim, calib.image_size):
            return px
    return true_px


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
    # Approach geometry, for the recorder's vision correction before the touch.
    touch_axis: tuple[float, float, float] = (0.0, 0.0, -1.0)  # pipette axis, out of the tip
    pretouch_back_mm: float = 0.0  # where the precision phase starts, back along the axis
    precision_speed_mm_s: float = 0.0
    dwell_frames: int = 0
    # Set when the path was planned to a decoy (see sample_decoy_px): target_px/target_xyz are then
    # the TRUE target the marker shows and the correction lands on; this is where the path aimed.
    decoy_px: tuple[float, float] | None = None
    # Set for recovery episodes (plan_recovery): the start is a pose part-way along a path aimed
    # at decoy_px; recovery["start_frac"] says how far along (0 = its start, 1 = its pre-touch).
    recovery: dict | None = None
    # Recovery episodes: the unrecorded path from a high start down to joints[0] (part of the
    # decoy's checked plan). The recorder moves the stage and grabs the detector background at
    # lead_in[0], with the tip out of view, then flies this before recording.
    lead_in: np.ndarray | None = None

    @property
    def start_joints(self) -> np.ndarray:
        return self.joints[0]

    @property
    def pretouch_index(self) -> int:
        """Index in ``joints`` of the last frame before the precision phase (at the pre-touch point)."""
        return self.phase_frames.get("coarse", 0) + self.phase_frames.get("gate", 0)

    @property
    def duration_s(self) -> float:
        return len(self.joints) / self.fps

    def meta(self) -> dict:
        """JSON-safe record for episodes_meta.jsonl (deliberately NOT dataset features)."""
        decoy = {} if self.decoy_px is None else {"decoy_px": [round(float(v), 2) for v in self.decoy_px]}
        if self.recovery is not None:
            decoy["recovery"] = self.recovery
        return {
            "seed": int(self.seed),
            "target_px": [round(float(v), 2) for v in self.target_px],
            **decoy,
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

    def sample_episode(self, seed: int, target_sampler=None, start_q=None) -> EpisodePlan:
        """``target_sampler(rng) -> px`` replaces the uniform target draw (initial and on
        replacement), e.g. to keep redrawn decoys around one true target. ``start_q`` fixes the
        start pose instead of sampling it (plan_recovery)."""
        rng = np.random.default_rng(seed)
        draw = target_sampler or self.sample_target_px
        self.rejections = {}
        # The target is drawn ONCE and kept while start/path are resampled. Redrawing it on
        # every rejection would make hard-to-reach parts of the image proportionally rarer in
        # the dataset (on Huanyu's setup acceptance ranges 22-59% across the image), so the
        # policy would see a skewed target distribution. Only a target that fails
        # `target_attempts` times in a row is replaced - i.e. one that is genuinely unreachable.
        target_px = draw(rng)
        for attempt in range(self.lim.max_attempts):
            if attempt and attempt % self.lim.target_attempts == 0:
                self.rejections["target replaced (unreachable)"] = (
                    self.rejections.get("target replaced (unreachable)", 0) + 1
                )
                target_px = draw(rng)
            try:
                plan = self._try_episode(rng, target_px, *([] if start_q is None else [start_q]))
                plan.seed = seed
                return plan
            except _RejectedSampleError as r:
                self.rejections[str(r)] = self.rejections.get(str(r), 0) + 1
        raise PlanningError(
            f"No feasible episode in {self.lim.max_attempts} attempts (seed {seed}). "
            f"Rejections: {self.rejections}. The limits in bench.yaml are infeasible together."
        )

    def sample_target_px(self, rng: np.random.Generator) -> np.ndarray:
        return sample_target_px(rng, self.lim, self.calib.image_size)

    def touch_point(self, target_px) -> np.ndarray:
        """Where the tip touches for a target pixel: the calibration's dish-plane point plus the
        bench's touch bias, at the surface plus the touch offset."""
        lim = self.lim
        xy = self.calib.pixel_to_xy(np.asarray(target_px, dtype=float))[0]
        return np.array(
            [xy[0] + lim.touch_bias_xy_mm[0], xy[1] + lim.touch_bias_xy_mm[1], self.calib.surface_z_mm + lim.touch_offset_mm]
        )

    def _try_episode(self, rng: np.random.Generator, target_px=None, start_q=None) -> EpisodePlan:
        lim = self.lim
        surface = self.calib.surface_z_mm

        # --- target and approach geometry
        if target_px is None:
            target_px = self.sample_target_px(rng)
        target_px = np.asarray(target_px, dtype=float)
        p_touch = self.touch_point(target_px)
        r_touch = random_rotation(rng, lim.touch_rot_jitter_deg) @ self.touch_nominal[:3, :3]
        axis = r_touch[:, 2]  # pipette axis, pointing out of the tip
        lateral = rng.normal(size=3)
        lateral -= axis * (lateral @ axis)
        lateral *= rng.uniform(0, lim.pretouch_lateral_mm) / max(np.linalg.norm(lateral), 1e-9)
        p_pre = p_touch - axis * rng.uniform(*lim.pretouch_back_mm) + lateral
        p_gate = p_touch - axis * rng.uniform(*lim.gate_back_mm)

        # --- start
        if start_q is None:
            p_start = self.start_nominal[:3, 3] + rng.uniform(-1, 1, 3) * np.asarray(lim.start_pos_jitter_mm)
            if p_start[2] < surface + lim.start_min_height_mm:
                raise _RejectedSampleError("start too low")
            r_start = random_rotation(rng, lim.start_rot_jitter_deg) @ self.start_nominal[:3, :3]
            q_start = self.kin.ik(make_transform(r_start, p_start), lim.start_nominal_joints)
            if q_start is None:
                raise _RejectedSampleError("start IK")
        else:
            q_start = np.asarray(start_q, dtype=float)
            t_start = self.kin.tip(q_start)
            p_start, r_start = t_start[:3, 3], t_start[:3, :3]

        # A recovery start already below the gate (part-way down a decoy's gate line) goes straight
        # to the pre-touch point: back up to the gate and down again is not a correction.
        below_gate = start_q is not None and float(np.dot(p_start - p_gate, axis)) > 0
        phases = [
            (
                "gate",
                np.vstack([p_start if below_gate else p_gate, p_pre]),
                r_start if below_gate else r_touch,
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

        if not below_gate:
            # --- coarse curve: start -> via(s) -> gate (a recovery start takes the direct curve)
            n_via = 0 if start_q is not None else int(rng.integers(lim.n_via[0], lim.n_via[1] + 1))
            fracs = np.sort(rng.uniform(0.25, 0.75, n_via))
            vias = [
                p_start + f * (p_gate - p_start) + rng.uniform(-1, 1, 3) * np.asarray(lim.via_jitter_mm)
                for f in fracs
            ]
            coarse_poly = catmull_rom(np.vstack([p_start, *vias, p_gate]))
            phases.insert(
                0, ("coarse", coarse_poly, r_start, r_touch, lim.coarse_speed_mm_s, lim.max_joint_step_deg, 0.0)
            )

        joints, precision, phase_frames, speeds = [q_start], [0.0], {}, {}
        for name, poly, r0, r1, speed, max_step, flag in phases:
            speeds[name] = rng.uniform(*speed)
            qs = self._time_and_solve(poly, r0, r1, speeds[name], max_step, joints[-1], name)
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
            touch_axis=tuple(float(v) for v in axis),
            pretouch_back_mm=float(np.dot(p_touch - p_pre, axis)),
            precision_speed_mm_s=float(speeds["precision"]),
            dwell_frames=n_dwell,
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

    # ------------------------------------------------------------------ near the dish

    def plan_line(
        self, q_from, p_to, speed_mm_s: float, name: str = "correction", max_step_deg: float | None = None
    ) -> np.ndarray:
        """Straight, slow tip move from ``q_from`` to ``p_to`` keeping the orientation, at the
        precision-phase joint-step limit (or ``max_step_deg``); collision-checked. For the
        recorder's vision correction: a sub-millimetre move between two points near the dish.
        Raises PlanningError."""
        q_from = np.asarray(q_from, float)
        t = self.kin.tip(q_from)
        poly = np.vstack([t[:3, 3], np.asarray(p_to, float)])
        step = self.lim.max_joint_step_precision_deg if max_step_deg is None else max_step_deg
        try:
            qs = self._time_and_solve(poly, t[:3, :3], t[:3, :3], speed_mm_s, step, q_from, name)
        except _RejectedSampleError as e:
            raise PlanningError(f"{name}: {e}") from e
        qs = np.array(qs)
        c = self.checker.check_path(np.vstack([q_from, qs]), self.lim.check_step_deg)
        if c.min_mm < 0:
            raise PlanningError(f"{name} would collide ({c.where} vs {c.obstacle}, {c.min_mm:.1f} mm)")
        return qs

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


def plan_with_decoy(
    planner: Planner, seed: int, decoy_max_mm: float, correction_slack_mm: float = 0.15
) -> EpisodePlan:
    """``planner.sample_episode(seed)``, but with the path aimed at a decoy up to ``decoy_max_mm``
    from the target (see sample_decoy_px). The returned plan's target_px / target_xyz are the TRUE
    target - what the marker shows and the vision correction lands on - and decoy_px is where the
    path aimed. The true target is drawn from its own stream (seed + 9_000_000), so it stays put
    while the planner redraws unreachable decoys or the recorder tries another stage position.
    ``decoy_max_mm <= 0`` is exactly ``sample_episode(seed)``.

    The planner only checks the path to the decoy, so the correction's move from the pre-touch
    point to the true target (and to points ``correction_slack_mm`` around it, for the shift the
    correction adds) is checked here; PlanningError if it would collide. Without this, a target
    by the dish wall planned fine and was discarded at the correction every time (2026-11-05:
    97 retries of one episode).
    """
    if decoy_max_mm <= 0:
        return planner.sample_episode(seed)
    true_px = planner.sample_target_px(np.random.default_rng(seed + 9_000_000))
    plan = planner.sample_episode(
        seed, target_sampler=lambda rng: sample_decoy_px(rng, true_px, decoy_max_mm, planner.lim, planner.calib)
    )
    plan.decoy_px = plan.target_px
    plan.target_px = (float(true_px[0]), float(true_px[1]))
    plan.target_xyz = tuple(float(v) for v in planner.touch_point(true_px))
    q_pre = plan.joints[plan.pretouch_index]
    s = correction_slack_mm
    for dx, dy in [(0, 0), (s, 0), (-s, 0), (0, s), (0, -s)]:
        planner.plan_line(q_pre, np.asarray(plan.target_xyz) + [dx, dy, 0.0], plan.precision_speed_mm_s, "corrected touch")
    return plan


def plan_recovery(
    planner: Planner,
    seed: int,
    offset_mm: tuple[float, float] = (1.0, 4.0),
    start_frac: tuple[float, float] = (0.5, 1.0),
    correction_slack_mm: float = 0.15,
) -> EpisodePlan:
    """A recovery episode: the arm starts OFF course and the recorded path brings it back.

    A path is planned to a decoy ``offset_mm`` from the true target (as plan_with_decoy), and the
    start is its pose ``start_frac`` of the way from its start to its pre-touch point: an approach
    already aimed 1-4 mm off, as the policies' own were on the bench (2026-11-05/06: aimed 2-3 mm
    off at precision entry, where the demos were never more than ~0.7 mm off). The recorder
    transits there unrecorded and records the teacher's path from that pose to the TRUE target.
    So the data holds corrections from the states a policy drifts into but never the drift
    itself, which recording would teach (the DART / DAgger point).
    """
    rng = np.random.default_rng(seed + 8_000_000)
    true_px = planner.sample_target_px(np.random.default_rng(seed + 9_000_000))
    decoy = planner.sample_episode(
        seed,
        target_sampler=lambda r: sample_decoy_px(r, true_px, offset_mm[1], planner.lim, planner.calib, offset_mm[0]),
    )
    frac = float(rng.uniform(*start_frac))
    k = int(round(frac * decoy.pretouch_index))
    plan = planner.sample_episode(seed + 7_000_000, target_sampler=lambda r: true_px, start_q=decoy.joints[k])
    plan.seed = seed
    plan.decoy_px = decoy.target_px
    xy = planner.calib.pixel_to_xy
    offset = float(np.linalg.norm(xy(np.asarray(decoy.target_px))[0] - xy(true_px)[0]))
    plan.recovery = {"start_frac": round(frac, 3), "decoy_offset_mm": round(offset, 3)}
    plan.lead_in = decoy.joints[: k + 1]
    q_pre = plan.joints[plan.pretouch_index]
    s = correction_slack_mm
    for dx, dy in [(0, 0), (s, 0), (-s, 0), (0, s), (0, -s)]:
        planner.plan_line(
            q_pre, np.asarray(plan.target_xyz) + [dx, dy, 0.0], plan.precision_speed_mm_s, "corrected touch"
        )
    return plan


def plan_hover(
    planner: Planner, seed: int, height_mm: tuple[float, float] = (0.5, 2.5), correction_slack_mm: float = 0.15
) -> EpisodePlan:
    """A hover-recovery episode: the tip starts straight above the ring and goes straight down.

    On 2026-11-06 both handover policies brought the tip's image onto the ring 0.45-1.5 mm above
    the dish and stopped there: in every demo the tip reaches the ring in the image only ~0.1-0.2 mm
    above the dish, just before the dwell, so "tip on ring" looked finished. This episode starts in
    that state - the touch orientation, ``height_mm`` straight above the touch point, reached
    unrecorded along a normal plan to its pre-touch point - and records a vertical descent with the
    orientation held, so the tip's image stays on the ring until it touches.

    The plan is the single hover frame with ``touch_axis`` vertical, so the recorder's vision
    correction (checkpoints and touch, all along touch_axis) carries out the descent.
    """
    rng = np.random.default_rng(seed + 8_000_000)
    true_px = planner.sample_target_px(np.random.default_rng(seed + 9_000_000))
    base = planner.sample_episode(seed, target_sampler=lambda r: true_px)
    k = base.pretouch_index
    p_touch = np.asarray(base.target_xyz, float)
    h = float(rng.uniform(*height_mm))
    lead = planner.plan_line(base.joints[k], p_touch + [0.0, 0.0, h], 3 * base.precision_speed_mm_s, "hover lead-in")
    q_hover = lead[-1]
    s = correction_slack_mm
    for dx, dy in [(0, 0), (s, 0), (-s, 0), (0, s), (0, -s)]:
        planner.plan_line(q_hover, p_touch + [dx, dy, 0.0], base.precision_speed_mm_s, "hover descent")
    plan = EpisodePlan(
        joints=np.asarray([q_hover]),
        precision=np.ones(1),
        fps=base.fps,
        seed=seed,
        target_px=(float(true_px[0]), float(true_px[1])),
        target_xyz=tuple(float(v) for v in p_touch),
        clearance=planner.checker.check_path(np.vstack([base.joints[: k + 1], lead]), planner.lim.check_step_deg),
        phase_frames={},
        touch_axis=(0.0, 0.0, -1.0),
        pretouch_back_mm=h,
        precision_speed_mm_s=base.precision_speed_mm_s,
        dwell_frames=base.dwell_frames,
    )
    plan.lead_in = np.vstack([base.joints[: k + 1], lead])
    plan.recovery = {"kind": "hover", "height_mm": round(h, 3)}
    return plan
