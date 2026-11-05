"""Rewrite a recorded (absolute joint target) dataset as image-only, tip-motion actions.

    python scripts/meca500/convert_to_tip_actions.py [--src NAME] [--force]

Writes AdamAxelrod/<src>_tip_delta:
- action = per-step tip motion in the camera's axes, [du, dv, dz, ru, rv, rz] in mm and rad
  (autocollect/tip_actions.py), plus precision.state left absolute. Forward increments, the
  last frame of each episode holds, as in convert_to_delta_actions.py.
- observation.state is removed, so the policy sees the image only. Zhao et al. 2025 found that
  removing the state only helped with end-effector actions; with joint actions it scored 0.

The source must hold absolute joint targets (record_auto.py's datasets do), not deltas.
Ends with a round trip: each episode's actions are integrated back through IK from its first
target, and the drift of the tip from the recorded path is reported.
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import pandas as pd

from lerobot.datasets.dataset_tools import recompute_stats, remove_feature
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.tip_actions import (
    ACTION_NAMES,
    camera_axes,
    joints_to_tip_actions,
    tip_action_to_joints,
)
from lerobot.utils.constants import HF_LEROBOT_HOME

RIG = "meca500_basler"
USER = "AdamAxelrod"
DEFAULT_SRC = "basler_stage_touch_decoy_2026-11"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--force", action="store_true", help="overwrite the destination if it exists")
    args = ap.parse_args()

    home = Path(HF_LEROBOT_HOME) / USER
    src, tmp, dst = home / args.src, home / f"{args.src}_tip_tmp", home / f"{args.src}_tip_delta"
    for p in (tmp, dst):
        if p.exists():
            if not args.force:
                raise SystemExit(f"{p} exists; pass --force to overwrite it.")
            shutil.rmtree(p)

    bench = BenchConfig.load(RIG)
    kin, axes = bench.kinematics(), camera_axes(bench.calibration())
    print(f"base X/Y -> image u/v:\n{np.round(axes, 4)}  (det {np.linalg.det(axes):+.0f})")

    info = json.loads((src / "meta/info.json").read_text())
    names = info["features"]["action"]["names"]
    if names[:6] != [f"joint_{i}.pos" for i in range(1, 7)] or names[6] != "precision.state":
        raise SystemExit(f"{src} actions are {names}, not 6 joint targets + precision.state.")

    # The raw frames are not needed for training.
    shutil.copytree(src, tmp, ignore=shutil.ignore_patterns("raw"))
    files = sorted((tmp / "data").rglob("*.parquet"))
    for f in files:
        df = pd.read_parquet(f)
        act = np.stack(df["action"].to_numpy()).astype(float)
        # Deltas are ~0.1 deg; absolute targets sit tens of degrees from zero.
        if np.abs(act[:, :6]).mean() < 5.0:
            raise SystemExit(f"{f} looks like joint DELTAS already; convert from the absolute dataset.")
        out = act.copy()
        ep = df["episode_index"].to_numpy()
        for e in np.unique(ep):
            m = np.flatnonzero(ep == e)
            out[m, :6] = joints_to_tip_actions(kin, act[m, :6], axes)
        df["action"] = list(out.astype(np.float32))
        df.to_parquet(f, index=False)
    info = json.loads((tmp / "meta/info.json").read_text())
    info["features"]["action"]["names"] = ACTION_NAMES
    (tmp / "meta/info.json").write_text(json.dumps(info, indent=4), newline="\n")
    print(f"rewrote {len(files)} parquet file(s)")

    remove_feature(
        LeRobotDataset(f"{USER}/{tmp.name}", root=tmp), "observation.state", output_dir=dst, repo_id=f"{USER}/{dst.name}"
    )
    shutil.rmtree(tmp)
    for extra in ("autocollect_episodes.jsonl", "autocollect_discards.jsonl", "autocollect/excluded_episodes.json"):
        if (src / extra).exists():
            (dst / extra).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src / extra, dst / extra)
    ds = LeRobotDataset(f"{USER}/{dst.name}", root=dst)
    recompute_stats(ds, skip_image_video=True)

    stats = json.loads((dst / "meta/stats.json").read_text())["action"]
    print(f"\n{dst}\n  features: {sorted(ds.meta.features)}")
    for i, n in enumerate(ACTION_NAMES):
        print(f"  {n:16s} mean {stats['mean'][i]:+.5f}  std {stats['std'][i]:.5f}  max|.| "
              f"{max(abs(stats['min'][i]), abs(stats['max'][i])):.4f}")

    # Round trip on the source: integrate the actions back through IK, as eval_auto does.
    df = pd.concat([pd.read_parquet(f) for f in sorted((src / "data").rglob("*.parquet"))])
    worst = []
    for e in sorted(df["episode_index"].unique())[::10]:
        q_rec = np.stack(df[df["episode_index"] == e]["action"].to_numpy()).astype(float)[:, :6]
        acts = joints_to_tip_actions(kin, q_rec, axes)
        q, t = q_rec[0].copy(), kin.tip(q_rec[0])
        err = 0.0
        for k in range(len(acts) - 1):
            t, q = tip_action_to_joints(kin, t, q, acts[k], axes)
            if q is None:
                raise SystemExit(f"round trip: IK failed in episode {e} at frame {k}")
            err = max(err, float(np.linalg.norm(kin.tip(q)[:3, 3] - kin.tip(q_rec[k + 1])[:3, 3])))
        worst.append(err)
    print(f"  round trip (every 10th episode): worst tip drift {max(worst) * 1000:.2f} um")


if __name__ == "__main__":
    main()
