"""Why does the finish policy settle 4-9 px off the ring? Bias or deadband (THEORIES.md theory 1).

    python scripts/meca500/probe_finish_fixed_point.py --model <pretrained_model dir> [--model ...]

On the 2026-11-12 bench runs the crop finish policies stopped moving sideways (<= 0.5 um/step)
while the detected tip was still 4-9 px (60-160 um) from the target, although they re-read the
image every 15 steps. Two explanations make different predictions:
    bias      the policy follows a moved ring (slope ~1 for shifts of a few px) but its own idea of
              "on target" is a few px from the detector's tip: at the settled frame the chunk is
              ~0 while the needed correction is not.
    deadband  the policy does not respond to ring offsets below several px at all: slope ~0 for
              small shifts, rising only for large ones.
For each sample (a camera frame without the ring, the joints, the target pixel and the detected
tip) the ring is drawn at the target and moved by d px in 4 directions, the 256 px crop is taken
around the moved ring exactly as TargetCropStep does live, and the policy predicts one chunk:
    slope(d)   (change in the executed displacement) . d / |d|^2 for the first n_exec actions
               (15, as on the bench); 1 = follows the moved ring fully, 0 = ignores it.
    implied    where the tip would end relative to the target after the unshifted chunk, px:
               detected error + chunk displacement. A policy that perceives the tip correctly
               drives this towards 0; a biased one leaves it where the bench trial ended.
Samples:
    bench   the last live frame each bench trial's policy saw (epNNN/live), where it had settled
    demo    the finish batch's near-hold raw frames (training distribution), every EVERY-th episode
Writes a CSV next to the first model and prints medians.
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
from probe_marker_shift import hold_frames, load_states  # noqa: E402

from lerobot.policies import prepare_observation_for_inference  # noqa: E402
from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.overlay import crop_at, draw_target  # noqa: E402
from lerobot.robots.meca500.autocollect.tip_actions import camera_axes  # noqa: E402
from lerobot.utils.constants import OBS_STR  # noqa: E402
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features  # noqa: E402

RIG = "meca500_basler"
ROOT = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"
DEMO_DATASET = "basler_stage_touch_finish_2026-11"
DEMO_STATES = "basler_stage_touch_finish_2026-11_js_tip_delta"
BENCH_RUNS = [
    "outputs/eval_auto/finish_2026-11_crop256_img_act_finish_zscripted_n15_20261112-161347",
    "outputs/eval_auto/finish_2026-11_crop256_js_act_finish_zscripted_n15_20261112-162348",
]
SHIFTS_PX = (2, 4, 8, 16)
N_EXEC = 15  # actions executed per chunk on the bench (N_ACTION_STEPS)
EVERY = 4


class Probe:
    def __init__(self, model: str):
        self.policy, self.pre, self.post = ev.load_policy(model)
        self.crop = self.policy.crop_px
        hw = {f"joint_{i}.pos": float for i in range(1, 7)}
        hw["microscope_cam"] = (self.crop, self.crop, 3)
        self.feats = hw_to_dataset_features(hw, OBS_STR, use_video=True)

    @torch.inference_mode()
    def chunk(self, q, img, target, style) -> np.ndarray:
        obs = {f"joint_{i + 1}.pos": float(q[i]) for i in range(6)}
        obs["microscope_cam"] = crop_at(draw_target(img, target, style), target, self.crop)
        frame = build_dataset_frame(self.feats, obs, prefix=OBS_STR)
        x = self.pre(prepare_observation_for_inference(frame, torch.device("cuda"), ev.TASK, "meca500"))
        self.policy.reset()
        return self.post(self.policy.predict_action_chunk(x)).squeeze(0).float().cpu().numpy()


def rgb(path) -> np.ndarray:
    return cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)


def bench_samples() -> list[dict]:
    out = []
    for run in BENCH_RUNS:
        for line in open(Path(run) / "results.jsonl"):
            r = json.loads(line)
            ep_dir = Path(run) / f"ep{r['episode']:03d}" / "live"
            idx = json.loads((ep_dir / "index.json").read_text())
            f = idx["frames"][-1]
            out.append({
                "set": "bench", "src": f"{Path(run).name.split('_act_')[0][-3:]}:ep{r['episode']}",
                "img": rgb(ep_dir / f["file"]), "q": np.asarray(f["joints_deg"], float),
                "target": np.asarray(r["target_px"], float), "tip": np.asarray(r["vision_tip_px"], float),
                "final_err_um": r["vision_error_um"],
            })
    return out


def demo_samples() -> list[dict]:
    raw = ROOT / DEMO_DATASET / "autocollect/raw"
    meta = [json.loads(x) for x in (ROOT / DEMO_DATASET / "autocollect_episodes.jsonl").read_text().splitlines() if x.strip()]
    states = load_states(ROOT / DEMO_STATES)
    out = []
    for m in meta[::EVERY]:
        ep = m["episode_index"]
        corr = [c for c in m.get("corrections", []) if c["checkpoint"] == "near" and c.get("detected_px")]
        holds = hold_frames(states[ep]) if ep in states else []
        if not corr or not holds or not (raw / f"ep{ep:04d}/near.png").exists():
            continue
        q = np.stack(states[ep]["observation.state"].values)[holds[-1]][:6].astype(float)
        out.append({
            "set": "demo", "src": f"finish:ep{ep}", "img": rgb(raw / f"ep{ep:04d}/near.png"), "q": q,
            "target": np.asarray(m["target_px"], float), "tip": np.asarray(corr[0]["detected_px"], float),
            "final_err_um": m.get("final_error_um"),
        })
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", action="append", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    bench = BenchConfig.load(RIG)
    calib, style = bench.calibration(), bench.target_style()
    axes = camera_axes(calib)
    A = np.asarray(calib.px_to_xy, float)[:, :2]  # mm per px (base XY)
    A_inv = np.linalg.inv(A)

    def disp_px(deltas: np.ndarray, n: int) -> np.ndarray:
        """Tip displacement of the first n tip actions, in image px."""
        return A_inv @ (axes.T @ deltas[:n, :2].sum(0))

    samples = bench_samples() + demo_samples()
    rows = []
    for model in args.model:
        probe = Probe(model)
        name = Path(model).parts[-4] if "checkpoints" in Path(model).parts else Path(model).name
        for s in samples:
            t, q, img = s["target"], s["q"], s["img"]
            base = probe.chunk(q, img, t, style)
            b15, b30 = disp_px(base, N_EXEC), disp_px(base, len(base))
            err = s["tip"] - t  # detected tip relative to the target, px
            row = {
                "model": name, "set": s["set"], "src": s["src"], "final_err_um": s["final_err_um"],
                "err_px": float(np.linalg.norm(err)), "move15_px": float(np.linalg.norm(b15)),
                "implied15_px": float(np.linalg.norm(err + b15)), "implied30_px": float(np.linalg.norm(err + b30)),
                # how much of the needed correction (-err) the unshifted chunk makes
                "corr_gain30": float(-(b30 @ err) / max(err @ err, 1e-9)),
            }
            for d in SHIFTS_PX:
                g = []
                for v in ((d, 0), (-d, 0), (0, d), (0, -d)):
                    v = np.asarray(v, float)
                    dd = disp_px(probe.chunk(q, img, t + v, style), len(base)) - b30
                    g.append(float(dd @ v / (v @ v)))
                row[f"slope_{d}px"] = float(np.mean(g))
            rows.append(row)
        del probe
        torch.cuda.empty_cache()

    df = pd.DataFrame(rows)
    out = Path(args.out) if args.out else Path(args.model[0]).parent / "probe_finish_fixed_point.csv"
    df.to_csv(out, index=False)
    cols = ["err_px", "move15_px", "implied30_px", "corr_gain30"] + [f"slope_{d}px" for d in SHIFTS_PX]
    pd.set_option("display.width", 200)
    print(df.groupby(["model", "set"])[cols].median().round(2).to_string())
    print("\nbench rows:")
    print(df[df.set == "bench"][["model", "src", "final_err_um", "err_px", "implied30_px", "corr_gain30"] + cols[-4:]].round(2).to_string(index=False))
    print(f"\nrows: {out}")


if __name__ == "__main__":
    main()
