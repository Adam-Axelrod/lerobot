"""Rewrite a Meca500 dataset's actions as per-step deltas.

WHY
---
The SpaceMouse teleop drives the arm with Cartesian velocity commands
(``MoveLinVelWrf``) on a background thread; its ``get_action()`` only reads
``GetJoints()`` back. So the "action" recorded during teleoperation is a position
READBACK, not a command. Measured on microscope_pipette_2026-09-10:

    action_t == observation.state_t exactly, in 97.4% of frames
    (94-99% at every speed, so it is structural, not tracking quality)

Two things follow, and both are bad:

1. A policy trained on that learns to predict where the arm already is. At
   deployment it commands the current pose, which is no motion at all. The
   2026-09-11 rollouts showed exactly this: 0.010 deg/step of undirected jitter
   (net 0.12 deg over 10 s against 2.9 deg of path length). The earlier run only
   moved because temporal ensembling was weighting 5-second-stale forecasts, i.e.
   the arm was being driven by prediction lag rather than by anything learned.

2. Under MEAN_STD normalization the fine phase is invisible to the loss. A
   demonstrated fine correction of 0.0045 deg is 1.8e-4 of the absolute action
   scale, against 2.4e-2 for coarse motion - a 130x ratio, so L1 over a 100-step
   chunk is ~99% approach.

Differencing the recorded positions recovers the control signal that was never
written down, and fixes both at once. Measured on the same dataset:

    representation    norm std   fine step   normalized   vs absolute
    absolute            25.537      0.0045     1.78e-04          1.0x
    per-step             0.297      0.0045     1.53e-02         85.9x
    rel-to-state        13.469      0.0045     3.37e-04          1.9x

Per-step deltas, not relative-to-state. With action chunking all 100 elements of a
chunk share one normalizer per joint; relative-to-state grows with k (element 99 is
5 s of travel), so its normalizer is set by the far end of the chunk and the near
end - the part actually executed - stays near zero. Worse here specifically: since
action == state, element 0 of a relative-to-state chunk is identically 0. That is
also why lerobot's built-in ``use_relative_actions`` (pi0/GR00T only, and
relative-to-state) would not help, quite apart from ACT not supporting it.

WHAT IT DOES
------------
Each joint channel becomes the FORWARD increment, per episode:

    D_k = A_{k+1} - A_k        for k = 0 .. N-2
    D_{N-1} = 0                (the demos all end held still, so 0 means "hold")

``precision.state`` is left absolute - it is a gate, not a position.

Forward rather than backward increments so that executing element 0 commands the
NEXT step of motion. A backward increment (A_k - A_{k-1}) is motion that has already
happened, and executing it would replay the trajectory one step late.

The schema is unchanged (still 7 float32 action dims with the same names), so only
the values and the statistics differ. Videos are copied untouched.

Deploy the result with ``Meca500Config(relative_actions=True)``, which integrates
each predicted delta onto the arm's current target pose.

USAGE
-----
    python scripts/meca500/convert_to_delta_actions.py
    python scripts/meca500/convert_to_delta_actions.py --src <repo_id> --dst <repo_id>
    python scripts/meca500/convert_to_delta_actions.py --check-only
"""

import argparse
import glob
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from lerobot.datasets.dataset_tools import recompute_stats
from lerobot.datasets.lerobot_dataset import LeRobotDataset

DEFAULT_SRC = "AdamAxelrod/microscope_pipette_2026-09-10"
DEFAULT_DST = "AdamAxelrod/microscope_pipette_2026-09-10_delta"

# Action channels that are gates rather than positions, and so are never differenced.
ABSOLUTE_CHANNELS = ("precision.state",)


def _root(repo_id: str) -> Path:
    from lerobot.utils.constants import HF_LEROBOT_HOME

    return Path(HF_LEROBOT_HOME) / repo_id


def _action_names(src: Path) -> list[str]:
    import json

    info = json.loads((src / "meta" / "info.json").read_text())
    names = info["features"]["action"].get("names")
    if not names:
        raise SystemExit(f"{src}/meta/info.json has no action feature names; cannot tell gates from joints.")
    return list(names)


def describe(repo_id: str) -> None:
    """Report the action/state relationship that motivates the conversion."""
    root = _root(repo_id)
    files = sorted(glob.glob(str(root / "data" / "**" / "*.parquet"), recursive=True))
    if not files:
        raise SystemExit(f"No parquet files under {root}/data - is {repo_id} downloaded?")
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    names = _action_names(root)
    joint_idx = [i for i, n in enumerate(names) if n not in ABSOLUTE_CHANNELS]

    act = np.stack(df["action"].to_numpy())[:, joint_idx]
    st = np.stack(df["observation.state"].to_numpy())[:, : len(joint_idx)]
    ep = df["episode_index"].to_numpy()

    same = np.abs(act - st).max(axis=1)
    d = np.zeros_like(act)
    for e in np.unique(ep):
        m = ep == e
        d[m] = np.vstack([np.diff(act[m], axis=0), np.zeros((1, len(joint_idx)))])

    print(f"  {repo_id}")
    print(f"    frames {len(df)}, episodes {len(np.unique(ep))}, action dims {len(names)} {names}")
    print(f"    action_t == state_t exactly in {100 * np.mean(same == 0):.1f}% of frames")
    print(f"    absolute action std (mean over joints): {act.std(axis=0).mean():8.3f} deg")
    print(f"    delta    action std (mean over joints): {d.std(axis=0).mean():8.3f} deg")
    print(f"    -> a 0.0045 deg fine step is {0.0045 / act.std(axis=0).mean():.2e} of the absolute scale, "
          f"{0.0045 / d.std(axis=0).mean():.2e} of the delta scale")


def convert(src_repo: str, dst_repo: str, force: bool = False) -> None:
    src, dst = _root(src_repo), _root(dst_repo)
    if not src.exists():
        raise SystemExit(f"Source dataset not found: {src}")
    if dst.exists():
        if not force:
            raise SystemExit(f"Destination already exists: {dst}\nPass --force to overwrite it.")
        print(f"  removing existing {dst}")
        shutil.rmtree(dst)

    names = _action_names(src)
    joint_idx = [i for i, n in enumerate(names) if n not in ABSOLUTE_CHANNELS]
    gate_idx = [i for i, n in enumerate(names) if n in ABSOLUTE_CHANNELS]
    print(f"  differencing {[names[i] for i in joint_idx]}")
    print(f"  leaving absolute {[names[i] for i in gate_idx] or '(none)'}")

    print(f"  copying {src} -> {dst}")
    shutil.copytree(src, dst)

    files = sorted(glob.glob(str(dst / "data" / "**" / "*.parquet"), recursive=True))
    if not files:
        raise SystemExit(f"No parquet files under {dst}/data")

    total, changed = 0, 0
    for f in files:
        df = pd.read_parquet(f)
        act = np.stack(df["action"].to_numpy()).astype(np.float32)
        ep = df["episode_index"].to_numpy()
        out = act.copy()
        for e in np.unique(ep):
            m = np.flatnonzero(ep == e)
            block = act[m][:, joint_idx]
            # Forward increments; the final frame of an episode holds (delta 0).
            out[m[:, None], joint_idx] = np.vstack(
                [np.diff(block, axis=0), np.zeros((1, len(joint_idx)), dtype=np.float32)]
            )
        df["action"] = list(out)
        df.to_parquet(f, index=False)
        total += len(df)
        changed += 1
    print(f"  rewrote {changed} parquet file(s), {total} frames")

    print("  recomputing statistics (this is what the normalizer reads)...")
    ds = LeRobotDataset(dst_repo)
    recompute_stats(ds, skip_image_video=True)

    print("\n  done. Verify:")
    describe(dst_repo)
    print(f"\n  Train on {dst_repo}, and deploy with Meca500Config(relative_actions=True).")
    print("  Those two MUST change together - a delta policy driven with relative_actions=False")
    print("  sends ~0.005 deg absolute joint targets and drives the arm into its lower limits.")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", default=DEFAULT_SRC)
    p.add_argument("--dst", default=DEFAULT_DST)
    p.add_argument("--force", action="store_true", help="overwrite the destination if it exists")
    p.add_argument("--check-only", action="store_true", help="report the action/state relationship, convert nothing")
    args = p.parse_args()

    if args.check_only:
        describe(args.src)
        return
    print("Before:")
    describe(args.src)
    print("\nConverting:")
    convert(args.src, args.dst, force=args.force)


if __name__ == "__main__":
    sys.exit(main())
