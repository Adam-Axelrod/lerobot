"""Can the policy's image pathway locate the tip relative to the ring at 50 um scale? THEORIES.md, theory 2.
Needs a GPU, no robot.

    python scripts/meca500/probe_tip_localisation.py [--quick]

Samples: every raw frame behind a detection in the recorded autocollect datasets
(autocollect/raw/epNNNN/{pretouch,mid,near,final}.png), with the target ring drawn on it exactly
as the policy sees it (bench target style). Label: our detector's tip pixel minus the target
pixel, from autocollect_episodes.jsonl (corrections[].detected_px, final.detected_px). Episodes in
autocollect/excluded_episodes.json are skipped, and so are final frames whose logged final error
exceeds FINAL_MAX_UM (detector misreads such as dataset_2026-11 ep 87).

Each variant regresses the tip-to-target vector with the same small head (1x1 conv to 32
channels, flatten, MLP), cross-validated in FOLDS folds grouped by episode:
    policy_frozen  the policy's own ResNet18 (--model), frozen, on the full frame through the
                   policy's own preprocessor; only the head trains
    full_tuned     ImageNet ResNet18 fine-tuned end to end on the full frame: what ACT's image
                   pathway can learn when this is its only job
    crop_tuned     the same on a CROP_PX full-resolution crop centred on the target
Reading the result (held-out error, um):
    policy_frozen bad, full_tuned good  -> the architecture can see it; the policy never learnt to
                                           look (theory 2, not theory 1)
    full_tuned bad, crop_tuned good     -> resolution / pooling is the bottleneck (theory 1)
    all bad                              -> labels or detector; check the per-sample CSV
Baselines: "zero" predicts the tip on the target (= the true offset), "mean" the training mean.

Writes outputs/probe_tip_localisation/<time>/: summary.txt and samples.csv (one row per held-out
prediction). --quick runs 1 fold, 2 epochs, on 120 samples, to check the plumbing.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import torch
import torchvision
from torch import nn
from torchvision.models._utils import IntermediateLayerGetter

sys.path.insert(0, str(Path(__file__).parent))
import eval_auto as ev  # noqa: E402

from lerobot.policies import prepare_observation_for_inference  # noqa: E402
from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.overlay import draw_target  # noqa: E402
from lerobot.utils.constants import OBS_STR  # noqa: E402
from lerobot.utils.feature_utils import build_dataset_frame, hw_to_dataset_features  # noqa: E402

RIG = "meca500_basler"
ROOT = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"
DATASETS = ["basler_stage_touch_dataset_2026-11", "basler_stage_touch_decoy_2026-11", "basler_stage_touch_recovery_2026-11"]
# The decoys + recovery ACT used for policy_asr (the paper's approach policy).
MODEL = "outputs/train/basler_stage_touch_decoyrec_2026-11_delta/checkpoints/100000/pretrained_model"
VARIANTS = ["policy_frozen", "full_tuned", "crop_tuned"]
FINAL_MAX_UM = 200.0
CROP_PX = 192  # +-96 px = +-1.3 mm around the target: holds the tip from "near" on, and most pretouch frames
FOLDS, EPOCHS, BATCH, SEED = 5, 40, 16, 0
LR_HEAD, LR_BACKBONE, WEIGHT_DECAY = 1e-3, 1e-4, 1e-4
LABEL_SCALE_PX = 100.0  # the head predicts offset / 100 px
IMAGENET_MEAN, IMAGENET_STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)
DEVICE = "cuda"


def load_samples() -> pd.DataFrame:
    rows = []
    for name in DATASETS:
        root = ROOT / name
        excl_file = root / "autocollect/excluded_episodes.json"
        excluded = set(json.loads(excl_file.read_text())["excluded"]) if excl_file.exists() else set()
        for line in (root / "autocollect_episodes.jsonl").read_text().splitlines():
            if not line.strip():
                continue
            r = json.loads(line)
            ep = r["episode_index"]
            if ep in excluded:
                continue
            checks = list(r.get("corrections") or [])
            if r.get("final") and (r.get("final_error_um") or 0) <= FINAL_MAX_UM:
                checks.append(dict(r["final"], checkpoint="final"))
            for c in checks:
                png = root / f"autocollect/raw/ep{ep:04d}/{c['checkpoint']}.png"
                if c.get("detected_px") is None or not png.exists():
                    continue
                rows.append({
                    "dataset": name, "episode": ep, "checkpoint": c["checkpoint"], "png": str(png),
                    "target_u": r["target_px"][0], "target_v": r["target_px"][1],
                    "tip_u": c["detected_px"][0], "tip_v": c["detected_px"][1],
                })
    df = pd.DataFrame(rows)
    df["du"], df["dv"] = df.tip_u - df.target_u, df.tip_v - df.target_v
    return df


def read_images(df: pd.DataFrame, style) -> list[np.ndarray]:
    """RGB, HxW = 640x427, ring drawn at the target."""
    return [
        draw_target(cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB), (u, v), style)
        for p, u, v in zip(df.png, df.target_u, df.target_v, strict=True)
    ]


def crop(img: np.ndarray, u: float, v: float) -> np.ndarray:
    h = CROP_PX // 2
    pad = cv2.copyMakeBorder(img, h, h, h, h, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cu, cv_ = int(round(u)), int(round(v))
    return pad[cv_ : cv_ + CROP_PX, cu : cu + CROP_PX]


def to_tensor(imgs: list[np.ndarray]) -> torch.Tensor:
    """uint8 NCHW on the CPU; normalised per batch on the GPU."""
    return torch.from_numpy(np.stack(imgs)).permute(0, 3, 1, 2).contiguous()


class Head(nn.Module):
    def __init__(self, feat_hw: tuple[int, int]):
        super().__init__()
        self.reduce = nn.Sequential(nn.Conv2d(512, 32, 1), nn.ReLU())
        self.mlp = nn.Sequential(nn.Flatten(), nn.Linear(32 * feat_hw[0] * feat_hw[1], 256), nn.ReLU(), nn.Linear(256, 2))

    def forward(self, f):
        return self.mlp(self.reduce(f))


def imagenet_backbone() -> nn.Module:
    m = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1)
    return IntermediateLayerGetter(m, return_layers={"layer4": "feature_map"})


class Tuned(nn.Module):
    def __init__(self, feat_hw):
        super().__init__()
        self.backbone, self.head = imagenet_backbone(), Head(feat_hw)
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))

    def forward(self, x_uint8):
        x = (x_uint8.float() / 255.0 - self.mean) / self.std
        return self.head(self.backbone(x)["feature_map"])


@torch.inference_mode()
def policy_features(model: str, imgs: list[np.ndarray]) -> torch.Tensor:
    """The policy's layer4 feature map for each image, through its own preprocessor (CPU, float16)."""
    policy, pre, _ = ev.load_policy(model)
    hw = {f"joint_{i}.pos": float for i in range(1, 7)}
    hw["microscope_cam"] = (640, 427, 3)
    feats_spec = hw_to_dataset_features(hw, OBS_STR, use_video=True)
    key = f"{OBS_STR}.images.microscope_cam"
    out = []
    for img in imgs:
        obs = {f"joint_{i}.pos": 0.0 for i in range(1, 7)}
        obs["microscope_cam"] = img
        x = pre(prepare_observation_for_inference(build_dataset_frame(feats_spec, obs, prefix=OBS_STR), torch.device(DEVICE), ev.TASK, "meca500"))
        out.append(policy.model.backbone(x[key])["feature_map"].half().cpu())
    del policy
    torch.cuda.empty_cache()
    return torch.cat(out)


def fit_predict(variant: str, x: torch.Tensor, y: np.ndarray, tr: np.ndarray, te: np.ndarray, epochs: int) -> np.ndarray:
    torch.manual_seed(SEED)
    feat_hw = tuple(x.shape[2:]) if variant == "policy_frozen" else ((x.shape[2] + 31) // 32, (x.shape[3] + 31) // 32)
    if variant == "policy_frozen":
        model = Head(feat_hw).to(DEVICE)
        params = [{"params": model.parameters(), "lr": LR_HEAD}]
    else:
        model = Tuned(feat_hw).to(DEVICE)
        params = [{"params": model.head.parameters(), "lr": LR_HEAD}, {"params": model.backbone.parameters(), "lr": LR_BACKBONE}]
    opt = torch.optim.AdamW(params, weight_decay=WEIGHT_DECAY)
    yt = torch.from_numpy(y / LABEL_SCALE_PX).float()
    rng = np.random.default_rng(SEED)
    for _ in range(epochs):
        model.train()
        for b in np.array_split(rng.permutation(tr), max(1, len(tr) // BATCH)):
            xb = x[b].to(DEVICE).float() if variant == "policy_frozen" else x[b].to(DEVICE)
            loss = nn.functional.smooth_l1_loss(model(xb), yt[b].to(DEVICE), beta=0.05)
            opt.zero_grad()
            loss.backward()
            opt.step()
    model.eval()
    preds = []
    with torch.inference_mode():
        for b in np.array_split(te, max(1, len(te) // BATCH)):
            xb = x[b].to(DEVICE).float() if variant == "policy_frozen" else x[b].to(DEVICE)
            preds.append(model(xb).float().cpu().numpy())
    return np.concatenate(preds) * LABEL_SCALE_PX


def summarise(res: pd.DataFrame, um_per_px: float) -> str:
    lines = []
    for (variant, cp), g in res.groupby(["variant", "checkpoint"]):
        e = g.err_px.to_numpy() * um_per_px
        lines.append(f"{variant:14s} {cp:9s} n {len(g):4d}  median {np.median(e):6.0f}  rms {np.sqrt(np.mean(e**2)):6.0f}  <=50 um {np.mean(e <= 50):5.0%}")
    for variant, g in res.groupby("variant"):
        e = g.err_px.to_numpy() * um_per_px
        lines.append(f"{variant:14s} {'ALL':9s} n {len(g):4d}  median {np.median(e):6.0f}  rms {np.sqrt(np.mean(e**2)):6.0f}  <=50 um {np.mean(e <= 50):5.0%}")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=MODEL, help="pretrained_model directory for policy_frozen")
    ap.add_argument("--variants", nargs="+", default=VARIANTS, choices=VARIANTS)
    ap.add_argument("--quick", action="store_true", help="1 fold, 2 epochs, 120 samples: plumbing check only")
    args = ap.parse_args()

    bench = BenchConfig.load(RIG)
    style, um_per_px = bench.target_style(), bench.calibration().mm_per_px * 1000
    df = load_samples()
    if args.quick:
        df = df.sample(120, random_state=SEED).reset_index(drop=True)
    folds, epochs = (1, 2) if args.quick else (FOLDS, EPOCHS)
    out_dir = Path("outputs/probe_tip_localisation") / (time.strftime("%Y%m%d-%H%M%S") + ("_quick" if args.quick else ""))
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"{len(df)} samples from {df.groupby(['dataset', 'episode']).ngroups} episodes; {um_per_px:.2f} um/px")
    print(df.groupby(["dataset", "checkpoint"]).size().to_string())

    # Folds grouped by episode, so no episode is in both training and test.
    groups = df.dataset + ":" + df.episode.astype(str)
    uniq = np.random.default_rng(SEED).permutation(groups.unique())
    fold_of = {g: i % FOLDS for i, g in enumerate(uniq)}
    fold = groups.map(fold_of).to_numpy()
    y = df[["du", "dv"]].to_numpy()

    imgs = read_images(df, style)
    results = []
    for variant in args.variants:
        t0 = time.time()
        if variant == "policy_frozen":
            x = policy_features(args.model, imgs)
        elif variant == "full_tuned":
            x = to_tensor(imgs)
        else:
            x = to_tensor([crop(im, u, v) for im, u, v in zip(imgs, df.target_u, df.target_v, strict=True)])
        for k in range(folds):
            tr, te = np.where(fold != k)[0], np.where(fold == k)[0]
            pred = fit_predict(variant, x, y, tr, te, epochs)
            base = {"zero": np.zeros_like(y[te]), "mean": np.broadcast_to(y[tr].mean(0), y[te].shape)}
            for name, p in [(variant, pred), *[(f"base_{n}", b) for n, b in base.items()]]:
                if name.startswith("base_") and variant != args.variants[0]:
                    continue
                part = df.iloc[te][["dataset", "episode", "checkpoint", "du", "dv"]].copy()
                part["variant"], part["fold"] = name, k
                part["pred_du"], part["pred_dv"] = p[:, 0], p[:, 1]
                part["err_px"] = np.hypot(part.pred_du - part.du, part.pred_dv - part.dv)
                results.append(part)
            e = np.hypot(*(pred - y[te]).T) * um_per_px
            print(f"{variant} fold {k}: median {np.median(e):.0f} um, rms {np.sqrt(np.mean(e**2)):.0f} um ({time.time() - t0:.0f} s)")
        del x
        torch.cuda.empty_cache()

    res = pd.concat(results)
    res["offset_um"] = np.hypot(res.du, res.dv) * um_per_px
    res["err_um"] = res.err_px * um_per_px
    res.to_csv(out_dir / "samples.csv", index=False)
    text = (
        f"probe_tip_localisation {time.strftime('%Y-%m-%d %H:%M')}  model {args.model}\n"
        f"{len(df)} samples, {folds} folds by episode, {epochs} epochs, crop {CROP_PX} px, {um_per_px:.2f} um/px\n"
        f"held-out error of the predicted tip-to-target vector (um):\n{summarise(res, um_per_px)}\n"
    )
    if "crop_tuned" in args.variants:
        c = res[res.variant == "crop_tuned"]
        inside = (c.du.abs() < CROP_PX / 2) & (c.dv.abs() < CROP_PX / 2)
        text += f"crop_tuned, tip inside the crop: median {np.median(c.err_um[inside]):.0f} um (n {inside.sum()}); outside: median {np.median(c.err_um[~inside]) if (~inside).any() else float('nan'):.0f} um (n {(~inside).sum()})\n"
    (out_dir / "summary.txt").write_text(text, encoding="utf-8")
    print(text)
    print(f"-> {out_dir}")


if __name__ == "__main__":
    main()
