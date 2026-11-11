"""Does a finish-only policy read the ring? THEORIES.md, theory 1, offline. Needs a GPU, no robot.

    python scripts/meca500/finish_policy_offline.py [--variants crop full] [--quick] [--shift-px N]

Trains a behaviour-cloning finish policy on the precision phase only (precision.state on) of
DATASET (decoys + recovery, actions as per-step tip motion along the image axes,
convert_to_tip_actions.py), with no joint state:
    input   crop  a CROP_PX full-resolution crop centred on the target, ring drawn (as recorded)
            full  the whole 640x427 frame (ACT's input), every FULL_STRIDE-th precision frame
    output  the next CHUNK steps of tip motion [du, dv, dz] (mm), as ACT's chunk; L1 on actions
            scaled by their per-dimension std over the precision frames
    model   ImageNet ResNet18 + MLP head, fine-tuned end to end
FOLDS folds grouped by episode. Every number below is on held-out episodes:
    chunk error       |predicted - recorded| XY tip displacement summed over the chunk (um), on all
                      precision frames and on the two still holds where the recorder corrects
                      (pretouch = first hold, near = last hold); baselines "zero" and "mean chunk"
    tracking slope    at those holds, how far the policy's chunk follows this episode's recorded
                      chunk beyond the average one: slope of (policy - mean) on (teacher - mean)
                      XY displacement. 1 = reproduces each episode's move, 0 = the average move
    gain ceiling      at those holds, the median share of the remaining XY path to the touch that
                      one recorded chunk covers: what a perfect ring-follower's marker gain reaches
    marker gain       the raw hold frame (autocollect/raw, no overlay) with the ring drawn at the
                      target and then moved SHIFT_PX along +-u and +-v (the crop moves with it);
                      gain = change in predicted XY displacement . shift / |shift|^2, as in
                      probe_marker_shift.py (ACT models: <= 0.12 at pretouch). 1 = follows fully.
A ring-reading finish policy shows a marker gain near the ceiling and a tracking slope near 1 at
the holds; a policy that replays an average finish shows ~0 for both, like the current models.

Writes outputs/finish_policy_offline/<time>/: summary.txt, frames.csv (held-out chunk errors),
holds.csv (per-hold slopes and gains). --quick: 1 fold, 1 epoch, 24 episodes (plumbing check).
"""

import argparse
import json
import time
from pathlib import Path

import av
import cv2
import numpy as np
import pandas as pd
import torch
import torchvision
from torch import nn

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.overlay import draw_target

RIG = "meca500_basler"
ROOT = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"
DATASET = "basler_stage_touch_decoyrec_2026-11_js_tip_delta"
CAM = "observation.images.microscope_cam"
VARIANTS = ["crop", "full"]
CROP_PX = 192
FULL_STRIDE = 3  # full frames are 0.8 MB each in RAM
CHUNK = 30  # ACT's chunk_size for these models: 1.5 s at 20 fps
MIN_HOLD = 4  # still frames that make a hold (probe_marker_shift.py)
STILL_MM = 1e-4
SHIFT_PX = 20  # 0.27 mm: inside what one chunk covers at both holds
FOLDS, EPOCHS, BATCH, SEED = 5, 10, 32, 0
LR_HEAD, LR_BACKBONE, WEIGHT_DECAY = 1e-3, 1e-4, 1e-4
IMAGENET_MEAN, IMAGENET_STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)
DEVICE = "cuda"


def load_episodes(root: Path, limit: int | None) -> tuple[pd.DataFrame, dict, pd.DataFrame]:
    data = pd.concat(pd.read_parquet(p) for p in sorted((root / "data").glob("*/*.parquet")))
    data = data.sort_values("index").reset_index(drop=True)
    meta = pd.concat(pd.read_parquet(p) for p in sorted((root / "meta/episodes").glob("*/*.parquet")))
    meta = meta.sort_values("episode_index").reset_index(drop=True)
    side = {json.loads(l)["episode_index"]: json.loads(l) for l in (root / "autocollect_episodes.jsonl").read_text().splitlines() if l.strip()}
    if limit:
        eps = sorted(np.random.default_rng(SEED).choice(meta.episode_index, limit, replace=False))
        data, meta = data[data.episode_index.isin(eps)], meta[meta.episode_index.isin(eps)]
    return data, side, meta


def episode_labels(g: pd.DataFrame) -> dict:
    """Precision frames, their action chunks and the two holds, for one episode (rows in order)."""
    act = np.stack(g.action.to_numpy())
    move, prec = act[:, :3], act[:, 6] > 0.5
    pad = np.vstack([move, np.zeros((CHUNK, 3))])
    chunks = np.stack([pad[i : i + CHUNK] for i in range(len(g))])
    still = np.all(np.abs(move) < STILL_MM, axis=1) & prec
    holds, i = [], 0
    while i < len(g):
        if still[i]:
            j = i
            while j < len(g) and still[j]:
                j += 1
            if j - i >= MIN_HOLD and j < len(g):  # a hold followed by motion (not the final dwell)
                holds.append(i)
            i = j
        else:
            i += 1
    return {"prec": np.where(prec)[0], "chunks": chunks, "holds": holds}


def decode(root: Path, meta: pd.DataFrame, want: dict[int, set[int]], fps: int, fn) -> dict[tuple[int, int], np.ndarray]:
    """fn(episode, RGB frame) for the wanted (episode, frame_index), decoding each video file once."""
    out = {}
    key = f"videos/{CAM}"
    for (chunk, file), m in meta.groupby([f"{key}/chunk_index", f"{key}/file_index"]):
        spans = [(r.episode_index, r[f"{key}/from_timestamp"], r.length) for _, r in m.iterrows()]
        path = root / f"videos/{CAM}/chunk-{chunk:03d}/file-{file:03d}.mp4"
        with av.open(str(path)) as c:
            for f in c.decode(video=0):
                t = float(f.pts * f.time_base)
                for ep, t0, n in spans:
                    k = int(round((t - t0) * fps))
                    if 0 <= k < n and k in want.get(ep, ()):
                        out[(ep, k)] = fn(ep, f.to_ndarray(format="rgb24"))
    return out


def crop(img: np.ndarray, u: float, v: float) -> np.ndarray:
    h = CROP_PX // 2
    pad = cv2.copyMakeBorder(img, h, h, h, h, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cu, cv_ = int(round(u)), int(round(v))
    return pad[cv_ : cv_ + CROP_PX, cu : cu + CROP_PX]


class Policy(nn.Module):
    def __init__(self, feat_hw):
        super().__init__()
        m = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1)
        self.backbone = nn.Sequential(*list(m.children())[:-2])
        self.head = nn.Sequential(
            nn.Conv2d(512, 32, 1), nn.ReLU(), nn.Flatten(),
            nn.Linear(32 * feat_hw[0] * feat_hw[1], 512), nn.ReLU(), nn.Linear(512, CHUNK * 3),
        )
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))

    def forward(self, x_uint8):
        x = (x_uint8.float() / 255.0 - self.mean) / self.std
        return self.head(self.backbone(x)).view(-1, CHUNK, 3)


def train(x: torch.Tensor, y: np.ndarray, scale: np.ndarray, tr: np.ndarray, epochs: int) -> Policy:
    torch.manual_seed(SEED)
    h, w = x.shape[2:]
    model = Policy(((h + 31) // 32, (w + 31) // 32)).to(DEVICE)
    opt = torch.optim.AdamW(
        [{"params": model.head.parameters(), "lr": LR_HEAD}, {"params": model.backbone.parameters(), "lr": LR_BACKBONE}],
        weight_decay=WEIGHT_DECAY,
    )
    yt = torch.from_numpy(y / scale).float()
    rng = np.random.default_rng(SEED)
    for _ in range(epochs):
        model.train()
        for b in np.array_split(rng.permutation(tr), max(1, len(tr) // BATCH)):
            loss = nn.functional.l1_loss(model(x[b].to(DEVICE)), yt[b].to(DEVICE))
            opt.zero_grad()
            loss.backward()
            opt.step()
    return model.eval()


@torch.inference_mode()
def predict(model: Policy, x: torch.Tensor, scale: np.ndarray) -> np.ndarray:
    """Predicted chunks (N, CHUNK, 3) in mm."""
    return np.concatenate([model(x[i : i + 64].to(DEVICE)).float().cpu().numpy() for i in range(0, len(x), 64)]) * scale


def raw_hold_frame(side: dict, ep: int, which: str) -> np.ndarray | None:
    """The source dataset's raw (no overlay) frame at that hold, found by seed."""
    src = side[ep].get("source")
    if not src:
        return None
    for line in (ROOT / src / "autocollect_episodes.jsonl").read_text().splitlines():
        r = json.loads(line)
        if r["seed"] == side[ep]["seed"]:
            p = ROOT / src / f"autocollect/raw/ep{r['episode_index']:04d}/{which}.png"
            return cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB) if p.exists() else None
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variants", nargs="+", default=VARIANTS, choices=VARIANTS)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--shift-px", type=int, default=SHIFT_PX, help="marker shift for the gain test (3 px = 40 um is inside the recorded corrections)")
    args = ap.parse_args()
    folds, epochs = (1, 1) if args.quick else (FOLDS, EPOCHS)

    bench = BenchConfig.load(RIG)
    style, mm_per_px = bench.target_style(), bench.calibration().mm_per_px
    root = ROOT / DATASET
    fps = json.loads((root / "meta/info.json").read_text())["fps"]
    data, side, meta = load_episodes(root, 24 if args.quick else None)
    out_dir = Path("outputs/finish_policy_offline") / (time.strftime("%Y%m%d-%H%M%S") + ("_quick" if args.quick else "") + (f"_shift{args.shift_px}" if args.shift_px != SHIFT_PX else ""))
    out_dir.mkdir(parents=True, exist_ok=True)

    lab = {ep: episode_labels(g) for ep, g in data.groupby("episode_index", sort=True)}
    eps = sorted(lab)
    fold_of = {ep: i % FOLDS for i, ep in enumerate(np.random.default_rng(SEED).permutation(eps))}
    allp = np.vstack([np.vstack(lab[e]["chunks"][lab[e]["prec"], 0]) for e in eps])
    scale = allp.std(0) + 1e-6
    print(f"{len(eps)} episodes, {sum(len(lab[e]['prec']) for e in eps)} precision frames, step std {np.round(scale * 1000, 2)} um")

    lines = [f"finish_policy_offline {time.strftime('%Y-%m-%d %H:%M')}  {DATASET}",
             f"{len(eps)} episodes, {folds} folds by episode, {epochs} epochs, chunk {CHUNK}, crop {CROP_PX} px, shift {args.shift_px} px = {args.shift_px * mm_per_px * 1000:.0f} um"]
    frame_rows, hold_rows = [], []
    for variant in args.variants:
        t0 = time.time()
        stride = FULL_STRIDE if variant == "full" else 1
        sel = {e: set(lab[e]["prec"][::stride].tolist()) | set(lab[e]["holds"]) for e in eps}
        frames = decode(root, meta, sel, fps, lambda ep, im: crop(im, *side[ep]["target_px"]) if variant == "crop" else im)
        keys = sorted(frames)
        x = torch.empty((len(keys), 3, *frames[keys[0]].shape[:2]), dtype=torch.uint8)
        for n, key in enumerate(keys):
            x[n] = torch.from_numpy(frames.pop(key)).permute(2, 0, 1)
        y = np.stack([lab[e]["chunks"][i] for e, i in keys])
        ep_of = np.array([k[0] for k in keys])
        is_hold = np.array([i in lab[e]["holds"] for e, i in keys])
        print(f"{variant}: {len(keys)} frames decoded ({time.time() - t0:.0f} s)")
        for k in range(folds):
            te_eps = [e for e in eps if fold_of[e] == k]
            tr = np.where([fold_of[e] != k for e in ep_of])[0]
            te = np.where(np.isin(ep_of, te_eps))[0]
            model = train(x, y, scale, tr, epochs)
            pred = predict(model, x[te], scale)
            mean_chunk = y[tr].mean(0)
            for i, j in enumerate(te):
                e, fi = keys[j]
                true_d, pred_d = y[j, :, :2].sum(0), pred[i, :, :2].sum(0)
                frame_rows.append({
                    "variant": variant, "fold": k, "episode": e, "frame": fi, "hold": bool(is_hold[j]),
                    "true_um": np.linalg.norm(true_d) * 1000, "err_um": np.linalg.norm(pred_d - true_d) * 1000,
                    "zero_um": np.linalg.norm(true_d) * 1000, "mean_um": np.linalg.norm(mean_chunk[:, :2].sum(0) - true_d) * 1000,
                })
            # The holds: offset slope (policy and teacher) and the marker gain on raw frames.
            for e in te_eps:
                holds = lab[e]["holds"]
                for which, fi in [("pretouch", holds[0]), ("near", holds[-1])] if len(holds) >= 2 else []:
                    j = keys.index((e, fi))
                    raw = raw_hold_frame(side, e, which)
                    row = {"variant": variant, "fold": k, "episode": e, "hold": which,
                           "teacher_mm": y[j, :, :2].sum(0).tolist(),
                           "policy_mm": pred[list(te).index(j), :, :2].sum(0).tolist(),
                           "remaining_mm": lab[e]["chunks"][fi:, 0, :2].sum(0).tolist(), "gain": None}
                    if raw is not None:
                        t = np.array(side[e]["target_px"], float)

                        def view(px, raw=raw):
                            im = draw_target(raw, tuple(px), style)
                            return crop(im, *px) if variant == "crop" else im

                        shifts = [np.array(s, float) for s in ((args.shift_px, 0), (-args.shift_px, 0), (0, args.shift_px), (0, -args.shift_px))]
                        batch = torch.from_numpy(np.stack([view(t)] + [view(t + s) for s in shifts])).permute(0, 3, 1, 2).contiguous()
                        d = predict(model, batch, scale)[:, :, :2].sum(1)
                        sm = [sh * mm_per_px for sh in shifts]
                        row["gain"] = float(np.mean([((d[n + 1] - d[0]) @ v) / (v @ v) for n, v in enumerate(sm)]))
                    hold_rows.append(row)
            print(f"{variant} fold {k} done ({time.time() - t0:.0f} s)")
            del model
            torch.cuda.empty_cache()
        del x

    fr, hr = pd.DataFrame(frame_rows), pd.DataFrame(hold_rows)
    fr.to_csv(out_dir / "frames.csv", index=False)
    hr.to_csv(out_dir / "holds.csv", index=False)
    lines.append("chunk XY displacement error, held out (um): median [policy | zero | mean chunk], over the true displacement")
    for (variant, hold), g in fr.groupby(["variant", "hold"]):
        lines.append(f"  {variant:5s} {'holds' if hold else 'other':5s} n {len(g):6d}  policy {g.err_um.median():6.0f}  zero {g.zero_um.median():6.0f}  mean {g.mean_um.median():6.0f}")
    lines.append("at the holds: tracking slope, gain ceiling and marker gain")
    for (variant, which), g in hr.groupby(["variant", "hold"]):
        t, pol, rem = (np.array(g[c].tolist()).reshape(-1, 2) for c in ("teacher_mm", "policy_mm", "remaining_mm"))
        dt, dp = t - t.mean(0), pol - t.mean(0)
        track = float((dp * dt).sum() / (dt * dt).sum())
        ceiling = float(np.median(np.linalg.norm(t, axis=1) / np.maximum(np.linalg.norm(rem, axis=1), 1e-6)))
        gains = g.gain.dropna()
        lines.append(f"  {variant:5s} {which:8s} n {len(g):4d}  tracking slope {track:5.2f}  gain ceiling {ceiling:4.2f}"
                     f"  | marker gain median {gains.median():5.2f} [IQR {gains.quantile(0.25):.2f}, {gains.quantile(0.75):.2f}] (n {len(gains)})")
    text = "\n".join(lines) + "\n"
    (out_dir / "summary.txt").write_text(text, encoding="utf-8")
    print(text)
    print(f"-> {out_dir}")


if __name__ == "__main__":
    main()
