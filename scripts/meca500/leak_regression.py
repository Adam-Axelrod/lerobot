"""How much does the joint state at the pre-touch point give away the target? Needs no robot or GPU.

    python scripts/meca500/leak_regression.py [DATASET ...]

For each recorded (absolute) autocollect dataset: the joint state at each episode's pre-touch
frame (the start of the first still hold in precision mode - the recorder's first correction pause) against the
episode's TRUE target in the dish plane, in robot X/Y mm (target_xyz_mm in
autocollect_episodes.jsonl: the target pixel through the scope calibration; the camera is fixed,
so this is where the marker is, whatever the stage does). Cross-validated (K folds by episode)
RMS error in um for:
    mean         predict the mean target (baseline: the target spread)
    joints       linear regression on the 6 joint angles
    joints+stage the same plus the stage X/Y
    tip          linear regression on the forward-kinematics tip X/Y/Z (the joints' own nonlinear map)
With the path aimed at the target, "tip" should be close to the label noise; with decoys it should
be about the decoy spread (RMS of a distance uniform on [0, 1.6] mm is 0.92 mm).
Episodes listed in autocollect/excluded_episodes.json are kept: this measures the data, not training.
Parquet files that are still being written (no footer) are skipped and reported.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow.parquet as pq

from lerobot.robots.meca500.autocollect.bench import BenchConfig

RIG = "meca500_basler"
ROOT = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"
DEFAULT = ["basler_stage_touch_dataset_2026-11", "basler_stage_touch_decoy_2026-11"]
K = 5
SEED = 0
MIN_HOLD = 4


def load_frames(root: Path) -> pd.DataFrame:
    parts = []
    for f in sorted((root / "data").glob("*/*.parquet")):
        try:
            parts.append(pq.read_table(f, columns=["action", "observation.state", "episode_index", "frame_index"]).to_pandas())
        except Exception:  # noqa: BLE001 - a file still open in the recorder
            print(f"  skipped (still being written?): {f.relative_to(root)}")
    return pd.concat(parts)


def pretouch_frame(g: pd.DataFrame) -> int | None:
    """First still run (>= MIN_HOLD frames) with the precision flag on: the pre-touch checkpoint
    hold. Without the flag test, pauses in the coarse transit (~28 mm up) were picked in 1 of 7."""
    act = np.stack(g["action"].values)
    a, precise = act[:, :6], act[:, 6] > 0.5
    still = np.r_[False, np.abs(np.diff(a, axis=0)).max(1) < 1e-6] & precise
    i, n = 0, len(a)
    while i < n:
        if still[i]:
            j = i
            while j < n and still[j]:
                j += 1
            if j < n and j - i >= MIN_HOLD:
                return i - 1  # the first frame of the hold (its pose was reached one frame earlier)
            i = j
        else:
            i += 1
    return None


def cv_rms_um(X: np.ndarray | None, y: np.ndarray) -> float:
    rng = np.random.default_rng(SEED)
    folds = np.array_split(rng.permutation(len(y)), K)
    err = []
    for f in folds:
        tr = np.setdiff1d(np.arange(len(y)), f)
        if X is None:
            pred = np.repeat(y[tr].mean(0, keepdims=True), len(f), 0)
        else:
            A = np.c_[X, np.ones(len(X))]
            w, *_ = np.linalg.lstsq(A[tr], y[tr], rcond=None)
            pred = A[f] @ w
        err.append(((pred - y[f]) ** 2).sum(1))
    return 1000 * float(np.sqrt(np.concatenate(err).mean()))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("datasets", nargs="*", default=DEFAULT)
    args = ap.parse_args()
    kin = BenchConfig.load(RIG).kinematics()

    for name in args.datasets:
        root = ROOT / name
        print(name)
        meta = {
            m["episode_index"]: m
            for m in (json.loads(line) for line in (root / "autocollect_episodes.jsonl").read_text().splitlines() if line.strip())
        }
        df = load_frames(root)
        rows = []
        for ep, g in df.groupby("episode_index"):
            g = g.sort_values("frame_index")
            i = pretouch_frame(g)
            if i is None or ep not in meta:
                continue
            q = np.stack(g["observation.state"].values)[i][:6].astype(float)
            m = meta[ep]
            rows.append((q, kin.tip(q)[:3, 3], m["stage_xy_mm"], m["target_xyz_mm"][:2], m.get("decoy_px") is not None))
        q = np.array([r[0] for r in rows])
        tip = np.array([r[1] for r in rows])
        stage = np.array([r[2] for r in rows], float)
        y = np.array([r[3] for r in rows], float)
        res = {
            "mean": cv_rms_um(None, y),
            "joints": cv_rms_um(q, y),
            "joints+stage": cv_rms_um(np.c_[q, stage], y),
            "tip": cv_rms_um(tip, y),
        }
        aim = 1000 * float(np.sqrt(((tip[:, :2] - y) ** 2).sum(1).mean()))
        print(f"  {len(rows)} episodes (decoys: {sum(r[4] for r in rows)}), {K}-fold CV RMS error in um:")
        print("  " + "  ".join(f"{k} {v:.0f}" for k, v in res.items()) + f"   | tip-to-target at pre-touch, no fit: {aim:.0f}")


if __name__ == "__main__":
    main()
