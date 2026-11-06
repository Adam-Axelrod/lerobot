"""Does a microscope policy read the drawn target? Moves the marker on real recorded frames and
measures how the predicted action chunk responds. Needs a GPU, no robot.

    python scripts/meca500/probe_marker_shift.py --model <pretrained_model dir> [--dataset NAME]

For every EVERY-th usable episode of the dataset (episodes in autocollect/excluded_episodes.json
are skipped) it takes three checkpoints, each a raw frame (no marker, no video compression) and the
joint state at that moment:
    start     autocollect/raw/epNNNN/background.png, frame 0 - tip out of view, the first observation
    pretouch  pretouch.png, at the first still hold (the recorder's first correction pause)
    near      near.png, at the last still hold (0.4 mm before the touch)
The marker is drawn at the episode's target, then moved SHIFT_PX along +u, -u, +v and -v. For each
image the policy predicts one chunk; its joint deltas are summed and turned into a tip XY
displacement with the forward kinematics.
    gain             (change in displacement) . shift / |shift|^2, averaged over the 4 directions.
                     1.0 = the chunk follows the moved marker fully; 0 = ignores it. One chunk only
                     covers chunk_mm, so near the dish a following policy is capped at about
                     chunk_mm / shift_mm.
    target_resp_mm   |change in displacement| for the moved marker (any direction)
    texture_resp_mm  same state and marker on another episode's dish image (the "background" check)
    no_marker_resp_mm  the marker removed
The medians per checkpoint are printed; per-row results go to --out.

2026-11-05, basler_stage_touch_2026-11_delta (no decoys), 40 episodes, 80 px = 1.07 mm:
median gain start 0.62, pretouch 0.014, near 0.071, while the texture swap moved the chunk as much
as the marker did. That is the "at most 7%" response near the dish.

Image-only tip-action policies (dataset *_tip_delta) are probed the same way: pass the source
dataset and its joint-delta copy (--dataset ..._decoy_2026-11 --delta-dataset ..._decoy_2026-11_delta)
for the frames and hold points; the joint state is then unused by the policy.
"""

import argparse
import glob
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import torch

sys.path.insert(0, str(Path(__file__).parent))
import eval_auto as ev  # noqa: E402

from lerobot.policies import prepare_observation_for_inference  # noqa: E402
from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.overlay import draw_target  # noqa: E402
from lerobot.robots.meca500.autocollect.tip_actions import camera_axes  # noqa: E402
from lerobot.utils.constants import OBS_STR  # noqa: E402
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features  # noqa: E402

RIG = "meca500_basler"
ROOT = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"
SHIFT_PX = 80
EVERY = 5
SEED = 0


def load_states(delta_root: Path) -> dict:
    files = glob.glob(str(delta_root / "data/*/*.parquet"))
    cols = ["observation.state", "action", "episode_index", "frame_index"]
    df = pd.concat(pd.read_parquet(f, columns=cols) for f in files)
    return {ep: g.sort_values("frame_index") for ep, g in df.groupby("episode_index")}


def hold_frames(g) -> list[int]:
    """First frame of each mid-episode still run of >= 4 frames (the correction holds), in order."""
    a = np.stack(g["action"].values)[:, :6]
    still = np.abs(a).max(1) < 1e-6
    out, i, n = [], 0, len(a)
    while i < n:
        if still[i]:
            j = i
            while j < n and still[j]:
                j += 1
            if j < n and j - i >= 4:
                out.append(i)
            i = j
        else:
            i += 1
    return out


class Probe:
    def __init__(self, model: str):
        self.policy, self.pre, self.post = ev.load_policy(model)
        hw = {f"joint_{i}.pos": float for i in range(1, 7)}
        hw["microscope_cam"] = (640, 427, 3)
        self.feats = hw_to_dataset_features(hw, OBS_STR, use_video=True)

    @torch.inference_mode()
    def chunk(self, q, img) -> np.ndarray:
        obs = {f"joint_{i + 1}.pos": float(q[i]) for i in range(6)}
        obs["microscope_cam"] = img
        frame = build_dataset_frame(self.feats, obs, prefix=OBS_STR)
        x = self.pre(prepare_observation_for_inference(frame, torch.device("cuda"), ev.TASK, "meca500"))
        return self.post(self.policy.predict_action_chunk(x)).squeeze(0).float().cpu().numpy()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help="pretrained_model directory")
    ap.add_argument("--dataset", default="basler_stage_touch_dataset_2026-11", help="recorded (absolute) dataset name")
    ap.add_argument("--delta-dataset", default=None, help="default: <dataset>_delta")
    ap.add_argument("--out", default=None, help="per-row CSV (default: <model>/../probe_marker_shift.csv)")
    args = ap.parse_args()

    raw_root = ROOT / args.dataset
    delta_root = ROOT / (args.delta_dataset or f"{args.dataset}_delta")
    raw = raw_root / "autocollect/raw"
    excl_file = raw_root / "autocollect/excluded_episodes.json"
    excluded = set(json.loads(excl_file.read_text())["excluded"]) if excl_file.exists() else set()

    bench = BenchConfig.load(RIG)
    kin, calib, style = bench.kinematics(), bench.calibration(), bench.target_style()
    meta = {
        json.loads(line)["episode_index"]: json.loads(line)
        for line in (raw_root / "autocollect_episodes.jsonl").read_text().splitlines()
        if line.strip()
    }
    states = load_states(delta_root)
    probe = Probe(args.model)
    rng = np.random.default_rng(SEED)
    eps = [e for e in sorted(meta) if e not in excluded and e in states][::EVERY]

    def read(ep: int, png: str) -> np.ndarray:
        return cv2.cvtColor(cv2.imread(str(raw / f"ep{ep:04d}" / png)), cv2.COLOR_BGR2RGB)

    axes = camera_axes(calib)

    def disp(q, deltas) -> np.ndarray:
        """Tip XY displacement (mm) after integrating the chunk's actions from q. Tip actions
        (image-only policies) add up directly in camera axes; map them back to base X/Y."""
        if probe.policy.tip_actions:
            return axes.T @ deltas[:, :2].sum(0)
        return kin.tip(q + deltas[:, :6].sum(0))[:3, 3][:2] - kin.tip(q)[:3, 3][:2]

    rows = []
    for ep in eps:
        g, target = states[ep], np.asarray(meta[ep]["target_px"])
        holds = hold_frames(g)
        checkpoints = {"start": (0, "background.png")}
        if holds:
            checkpoints["pretouch"] = (holds[0], "pretouch.png")
            checkpoints["near"] = (holds[-1], "near.png")
        for name, (fi, png) in checkpoints.items():
            img = read(ep, png)
            q = np.stack(g["observation.state"].values)[fi][:6].astype(float)
            base = disp(q, probe.chunk(q, draw_target(img, target, style)))
            resp = []
            for d in ((SHIFT_PX, 0), (-SHIFT_PX, 0), (0, SHIFT_PX), (0, -SHIFT_PX)):
                t2 = target + np.asarray(d)
                shift_mm = calib.pixel_to_xy(t2)[0] - calib.pixel_to_xy(target)[0]
                dd = disp(q, probe.chunk(q, draw_target(img, t2, style))) - base
                resp.append((float(dd @ shift_mm / (shift_mm @ shift_mm)), float(np.linalg.norm(dd))))
            other = eps[(eps.index(ep) + 1 + rng.integers(len(eps) - 1)) % len(eps)]
            tex = float(np.linalg.norm(disp(q, probe.chunk(q, draw_target(read(other, png), target, style))) - base))
            nomark = float(np.linalg.norm(disp(q, probe.chunk(q, img)) - base))
            rows.append({
                "ep": ep, "cp": name, "chunk_mm": float(np.linalg.norm(base)),
                "gain": float(np.mean([r[0] for r in resp])), "target_resp_mm": float(np.mean([r[1] for r in resp])),
                "texture_resp_mm": tex, "no_marker_resp_mm": nomark,
            })

    df = pd.DataFrame(rows)
    out = Path(args.out) if args.out else Path(args.model).parent / "probe_marker_shift.csv"
    df.to_csv(out, index=False)
    cols = ["chunk_mm", "gain", "target_resp_mm", "texture_resp_mm", "no_marker_resp_mm"]
    print(f"{args.model}\n{len(eps)} episodes of {args.dataset}; marker moved {SHIFT_PX} px = {SHIFT_PX * calib.mm_per_px:.2f} mm")
    print(df.groupby("cp")[cols].median().round(3).to_string())
    print(f"rows: {out}")


if __name__ == "__main__":
    main()
