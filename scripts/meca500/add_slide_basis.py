"""Add each frame's slide basis to a joint-state dataset, for ACT's state_slide_mm augmentation.

    python scripts/meca500/add_slide_basis.py [--src NAME] [--force]

Writes AdamAxelrod/<src>_sb: the same dataset plus a "slide_basis" column (12 floats, not a policy
input). Row j, column k of its 6x2 reshape is joint j's change in degrees per mm of tip translation
along robot x (k = 0) or y (k = 1), orientation held: the columns of J^-1 for the tip's
translation. A robot base slid by d leaves the tip where it is but shows the joints that would put
it d away, q - B d (to first order; autocollect/asr_bridge.VirtualSlide does the exact IK). Training
on q - B d with a random d per sample means the joints no longer say where the tip is over the
dish; only the image does.

Ends with a check of the linearisation against VirtualSlide's IK at 4 mm on sampled frames.
"""

import argparse
import shutil

import numpy as np
import pandas as pd

from lerobot.datasets.dataset_tools import add_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.meca500.autocollect.asr_bridge import VirtualSlide
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.utils.constants import HF_LEROBOT_HOME

RIG = "meca500_basler"
USER = "AdamAxelrod"
DEFAULT_SRC = "basler_stage_touch_decoyrec_2026-11_delta"
CHECK_MM = 4.0


def slide_basis(kin, q_deg) -> np.ndarray:
    """(6, 2) joint degrees per mm of tip translation along robot x and y."""
    jac = kin.jacobian(q_deg)  # rows [v (mm/rad); w (rad/rad)]
    rhs = np.zeros((6, 2))
    rhs[0, 0] = rhs[1, 1] = 1.0
    return np.degrees(np.linalg.solve(jac, rhs))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--force", action="store_true", help="replace an existing <src>_sb")
    args = ap.parse_args()
    src_id, dst_id = f"{USER}/{args.src}", f"{USER}/{args.src}_sb"
    dst_root = HF_LEROBOT_HOME / dst_id
    if dst_root.exists():
        if not args.force:
            raise SystemExit(f"{dst_root} exists; --force replaces it")
        shutil.rmtree(dst_root)

    kin = BenchConfig.load(RIG).kinematics()
    ds = LeRobotDataset(src_id)
    files = sorted((ds.root / "data").glob("*/*.parquet"))
    q = np.concatenate([np.stack(pd.read_parquet(f, columns=["observation.state"])["observation.state"]) for f in files])
    basis = np.stack([slide_basis(kin, qi).reshape(12) for qi in q]).astype(np.float32)
    print(f"{len(q)} frames; |B| per joint (deg/mm) median {np.round(np.median(np.abs(basis), 0), 3).tolist()}")

    # The linearisation against the exact virtual joints, at CHECK_MM in a random direction.
    rng = np.random.default_rng(0)
    errs = []
    for i in rng.choice(len(q), 200, replace=False):
        a = rng.uniform(0, 2 * np.pi)
        d = CHECK_MM * np.array([np.cos(a), np.sin(a)])
        exact = VirtualSlide(kin, d).to_virtual(q[i])
        if exact is not None:
            errs.append(np.abs(q[i] - basis[i].reshape(6, 2) @ d - exact).max())
    errs = np.array(errs)
    print(f"linear vs IK at {CHECK_MM} mm: max joint error median {np.median(errs):.4f} deg, max {errs.max():.4f} deg")

    out = add_features(
        ds,
        {"slide_basis": (basis, {"dtype": "float32", "shape": [12], "names": None})},
        output_dir=dst_root,
        repo_id=dst_id,
    )
    for side in ("autocollect", "autocollect_episodes.jsonl"):
        src = ds.root / side
        if src.is_dir():
            shutil.copytree(src, dst_root / side)
        elif src.exists():
            shutil.copy2(src, dst_root / side)
    print(f"wrote {dst_id}: {out.meta.total_episodes} episodes, {out.meta.total_frames} frames -> {dst_root}")


if __name__ == "__main__":
    main()
