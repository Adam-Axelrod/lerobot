"""Search for collision-free "tip on the optical axis" poses for a bench. No robot needed.

The planner needs ``approach.touch_nominal_joints``: a natural pose with the tip touching
the dish. Rather than guess, this sweeps the pipette's approach direction (azimuth and
elevation) and its roll about its own axis, with the tip fixed on the optical axis at the
dish surface. It solves IK for each candidate and ranks the reachable ones by clearance to
the obstacles (the tip's own exempt length excluded). The best few are printed ready to
paste into bench.yaml.

The dish height comes from the scope calibration once it exists; until then
SURFACE_Z_FALLBACK is used, so treat the result as provisional and re-run after
calibrate_scope.py. Re-check the chosen pose on the arm with hover-mode recording before
touching anything.
"""

import itertools
import sys

import numpy as np
from rigs import require_home_joints

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.kinematics import axis_angle, make_transform, matrix_to_pose

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
OPTICAL_AXIS_XY = (237.5, -85.5)
SURFACE_Z_FALLBACK = 32.5  # stage top (31) + ~1.5 mm dish bottom; replaced by the calibration
ELEVATIONS_DEG = range(15, 76, 10)  # pipette angle below horizontal
AZIMUTHS_DEG = range(0, 360, 20)  # direction the pipette points, in the base frame
ROLLS_DEG = range(0, 360, 30)  # rotation of the tool about the pipette axis
SHOW = 8
# The tip must be able to back out this far along the pipette from the touch pose - the
# planner's straight final approach - staying clear and this far inside every joint limit.
# A touch pose that is clear on its own but sits against a joint limit is unreachable.
CORRIDOR_MM = 60.0
CORRIDOR_STEP_MM = 3.0
JOINT_LIMIT_MARGIN_DEG = 8.0
# -----------------------------------------------------------------


def corridor_ok(kin, checker, target, q) -> bool:
    """Back the tip out along the pipette axis, IK seeded step to step; all clear and off-limits."""
    axis = target[:3, 2]
    for d in np.arange(CORRIDOR_STEP_MM, CORRIDOR_MM + 1e-9, CORRIDOR_STEP_MM):
        t = target.copy()
        t[:3, 3] = target[:3, 3] - axis * d
        q = kin.ik(t, q, max_iters=100)
        if q is None:
            return False
        if np.any(q < kin.lower + JOINT_LIMIT_MARGIN_DEG) or np.any(q > kin.upper - JOINT_LIMIT_MARGIN_DEG):
            return False
        if checker.clearance(q).min_mm < 0:
            return False
    return True


def suggest_start(kin, checker, q_touch, max_lift_mm: float = 150.0, step_mm: float = 3.0):
    """Walk from the touch pose: back out along the pipette, then straight up, IK seeded step to
    step so the start is on the touch pose's IK branch. Returns the highest clear pose reached
    (joints, lift, clearance) or None."""
    t = kin.tip(q_touch)
    axis = t[:3, 2]
    q = q_touch
    for d in np.arange(step_mm, CORRIDOR_MM + 1e-9, step_mm):
        tt = t.copy()
        tt[:3, 3] = t[:3, 3] - axis * d
        q = kin.ik(tt, q, max_iters=100)
        if q is None:
            return None
    base = t[:3, 3] - axis * CORRIDOR_MM
    best = None
    for lift in np.arange(step_mm, max_lift_mm + 1e-9, step_mm):
        tt = t.copy()
        tt[:3, 3] = base + [0, 0, lift]
        q = kin.ik(tt, q, max_iters=100)
        if (
            q is None
            or np.any(q < kin.lower + JOINT_LIMIT_MARGIN_DEG)
            or np.any(q > kin.upper - JOINT_LIMIT_MARGIN_DEG)
        ):
            break
        c = checker.clearance(q).min_mm
        if c < 0:
            break
        best = (q, lift, c)
    return best


def main() -> None:
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()
    checker = bench.collision_checker(kin)
    try:
        surface = bench.calibration().surface_z_mm
        print(f"Dish surface from the scope calibration: z = {surface:.3f} mm")
    except FileNotFoundError:
        surface = SURFACE_Z_FALLBACK
        print(f"No scope calibration yet - assuming the dish surface at z = {surface} mm (PROVISIONAL).")
    p_tip = np.array([*OPTICAL_AXIS_XY, surface])
    home = np.asarray(require_home_joints(RIG), float)
    seeds = [home, np.array([-60.0, 45, 20, 120, 70, -120]), np.array([-30.0, 30, 30, 0, 60, 0]), np.zeros(6)]

    results = []
    n = 0
    for el, az, roll in itertools.product(ELEVATIONS_DEG, AZIMUTHS_DEG, ROLLS_DEG):
        n += 1
        e, a = np.radians(el), np.radians(az)
        z = np.array(
            [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), -np.sin(e)]
        )  # pipette axis, into the dish
        x0 = np.cross([0, 0, 1], z)
        x0 /= np.linalg.norm(x0)
        x = axis_angle(z, np.radians(roll)) @ x0
        r = np.column_stack([x, np.cross(z, x), z])
        target = make_transform(r, p_tip)
        for seed in seeds:
            q = kin.ik(target, seed, max_iters=150)
            if q is None or abs(q[4]) < 10:
                continue
            c = checker.clearance(q)
            if c.min_mm >= 0 and corridor_ok(kin, checker, target, q):
                results.append((c.min_mm, el, az, roll, q, c))
            break
        print(f"\r  {n} candidates, {len(results)} clear", end="", flush=True)
    print()
    if not results:
        sys.exit("No collision-free touch pose found - check the obstacles, margins and tool.")

    # Keep the wrist on the same side as home: crossing J5 = 0 (the wrist singularity) is
    # something the planner refuses, so a touch pose on the other side could never be reached.
    same = [r for r in results if np.sign(r[4][4]) == np.sign(home[4])]
    if same:
        results = same
    else:
        print("WARNING: no clear pose with J5 on the same side as home - the planner will reject these.")
    results.sort(key=lambda r: -r[0])
    print(
        f"\nTop {SHOW} by clearance beyond the margins (links {checker.link_margin_mm}, tool {checker.tool_margin_mm} mm):"
    )
    for clr, el, az, roll, q, c in results[:SHOW]:
        print(
            f"  {clr:5.1f} mm ({c.where} vs {c.obstacle})  elevation {el:2d}  azimuth {az:3d}  roll {roll:3d}  "
            f"joints {[round(float(v), 2) for v in q]}"
        )
    best = results[0]
    print("\nPaste into bench.yaml (approach.touch_nominal_joints), PROVISIONAL until checked on the arm:")
    print(f"  touch_nominal_joints: {[round(float(v), 2) for v in best[4]]}")
    print(f"  tip pose: {np.round(matrix_to_pose(kin.tip(best[4])), 2).tolist()}")

    start = suggest_start(kin, checker, best[4])
    if start is None:
        print("\nNo start pose found by backing out and lifting from that touch pose.")
    else:
        q, lift, c = start
        print(
            f"\nMatching start (tip backed out {CORRIDOR_MM:.0f} mm along the pipette, lifted {lift:.0f} mm; "
            f"same IK branch as the touch pose, {c:.1f} mm clear):"
        )
        print(f"  start.nominal_joints: {[round(float(v), 2) for v in q]}")


if __name__ == "__main__":
    main()
