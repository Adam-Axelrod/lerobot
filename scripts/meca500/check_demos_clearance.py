"""Use the human demos to check the bench's collision model and to seed bench.yaml.

Two jobs, no robot needed:

1. **Model check.** Every frame of every human demo is a configuration the arm really
   occupied without hitting anything, so the collision checker must find NONE of them
   inside an obstacle. A hit means the CAD/registration is wrong (or the margin is larger
   than what the operator actually left) - fix that before trusting any planned path.
   Skipped, with a note, while bench.yaml has no obstacles yet.

2. **Seeding the limits.** Prints the demos' start and end poses so ``start.nominal_joints``
   and ``approach.touch_nominal_joints`` in bench.yaml come from what an operator actually
   did, plus the spread of the end points (how big a target area the demos covered).

Workflow: set RIG / DATASET below, then `python check_demos_clearance.py`.
"""

import glob
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.kinematics import Meca500Kinematics, matrix_to_pose
from lerobot.utils.constants import HF_LEROBOT_HOME

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
DATASET = "AdamAxelrod/basler_pipette_2026-09-16"
# Check every Nth frame (1 = all). Consecutive frames are nearly identical, so 2-5 loses
# nothing and keeps the run to seconds.
FRAME_STRIDE = 2
# Include the bench's tool (holder + pipette) in the check. Only valid if the demos were
# recorded with the SAME tool - Huanyu's basler_pipette_2026-09-16 used a different
# pipette, so for that dataset only the arm links are checked.
CHECK_TOOL = False
# -----------------------------------------------------------------


def load_states(repo_id: str) -> pd.DataFrame:
    root = HF_LEROBOT_HOME / repo_id
    files = sorted(glob.glob(str(root / "data" / "*" / "*.parquet")))
    if not files:
        sys.exit(f"No parquet files under {root}. Download the dataset first.")
    df = pd.concat(
        pd.read_parquet(f, columns=["observation.state", "episode_index", "frame_index"]) for f in files
    )
    return df.sort_values(["episode_index", "frame_index"]).reset_index(drop=True)


def fmt(v) -> str:
    return "[" + ", ".join(f"{x:.2f}" for x in v) + "]"


def main() -> None:
    bench = BenchConfig.load(RIG)
    try:
        kin = bench.kinematics()
        frame_name = "tip"
    except ValueError:
        kin = Meca500Kinematics()
        frame_name = "flange (tool.tip_pose_in_flange not set yet)"

    df = load_states(DATASET)
    q = np.stack(df["observation.state"].to_numpy())[:, :6]
    ep = df["episode_index"].to_numpy()
    first = np.r_[True, ep[1:] != ep[:-1]]
    last = np.r_[ep[1:] != ep[:-1], True]
    print(f"{DATASET}: {ep.max() + 1} episodes, {len(q)} frames\n")

    # ---- starts and ends
    for label, mask in (("START", first), ("END", last)):
        qs = q[mask]
        poses = np.array([matrix_to_pose(kin.tip(x) if frame_name == "tip" else kin.flange(x)) for x in qs])
        print(f"{label} poses ({frame_name}), over {len(qs)} episodes:")
        print(f"  median joints   {fmt(np.median(qs, axis=0))}")
        print(f"  joint std       {fmt(qs.std(axis=0))}")
        print(f"  median pose     {fmt(np.median(poses, axis=0))}   (x y z mm, a b g deg)")
        print(f"  pose std        {fmt(poses.std(axis=0))}")
        spread = poses[:, :3].max(axis=0) - poses[:, :3].min(axis=0)
        print(f"  xyz extent (mm) {fmt(spread)}\n")
    # ---- pivot estimate of the tip
    # Every demo ends with the tip on a dot inside a ~1-2 mm field of view, approached at
    # varied orientations, so the end flange poses are close to a pivot calibration: solve
    # R_i t + p_i = c for the tip offset t (flange frame) and the common point c (base frame).
    # Only as good as "every demo ended touching the same spot" (~mm), so it is a cross-check
    # for tool.tip_pose_in_flange from the CAD, never a replacement for it.
    flanges = [kin.flange(x) for x in q[last]]
    a = np.vstack([np.hstack([f[:3, :3], -np.eye(3)]) for f in flanges])
    b = np.concatenate([-f[:3, 3] for f in flanges])
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    tips = np.array([f[:3, :3] @ sol[:3] + f[:3, 3] for f in flanges])
    print("Pivot estimate from the demo end poses - the tip of the pipette USED IN THESE DEMOS (~mm):")
    print(f"  tip in flange frame  {fmt(sol[:3])} mm  (|t| = {np.linalg.norm(sol[:3]):.1f} mm)")
    print(f"  touch point (BRF)    {fmt(sol[3:])} mm  - z is roughly the dish surface")
    print(f"  residual tip spread  {fmt(tips.std(axis=0))} mm std\n")

    medoid_end = q[last][np.argmin(np.linalg.norm(q[last] - np.median(q[last], axis=0), axis=1))]
    print("Suggested bench.yaml values (an actual demo pose, not an average of poses):")
    print(f"  start.nominal_joints:          {fmt(np.median(q[first], axis=0))}")
    print(f"  approach.touch_nominal_joints: {fmt(medoid_end)}\n")

    # ---- model check
    try:
        checker = bench.collision_checker(kin)
        if not CHECK_TOOL:
            checker.points.tool_points = None
    except (ValueError, FileNotFoundError) as e:
        print(f"Collision check skipped: {e}")
        return
    sel = np.arange(0, len(q), FRAME_STRIDE)
    worst, hits = None, []
    for i in sel:
        c = checker.clearance(q[i])
        if worst is None or c.min_mm < worst[1].min_mm:
            worst = (i, c)
        if c.min_mm < 0:
            hits.append((i, c))
    i, c = worst
    print(
        f"Checked {len(sel)} demo frames. Tightest: episode {ep[i]} frame {df['frame_index'][i]}, "
        f"{c.where} vs {c.obstacle}, {c.min_mm:+.1f} mm beyond the margin."
    )
    if hits:
        print(f"\n!! {len(hits)} demo frames are INSIDE the margin - the model disagrees with reality.")
        for i, c in hits[:10]:
            print(
                f"   episode {ep[i]} frame {df['frame_index'][i]}: {c.where} vs {c.obstacle} ({c.min_mm:+.1f} mm)"
            )
        sys.exit(1)
    print("OK - no human demo frame collides with the model.")


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).parent))
    main()
