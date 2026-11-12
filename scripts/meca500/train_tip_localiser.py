"""Train the learned tip localiser for the finish refinement (THEORIES.md theory 1, 2026-11-12).

    python scripts/meca500/train_tip_localiser.py [--quick]

The 2026-11-12 bench session and probe_finish_fixed_point.py showed the ACT finish policy settles
where its own, implicit tip-to-ring estimate is zero, and that estimate is off by 1-8 px
(median ~3 px, 40 um) in a different direction each trial. probe_tip_localisation.py showed a
ResNet18 trained only to regress the tip-to-target vector on a 192 px crop gets 17 um median at
the near checkpoint. This trains that regressor on every detection frame of the recorded datasets
(including the finish batch), for eval_auto's REFINE step: at a hover just above the dish the
policy hands the last lateral correction to this network.

Same samples, crop, network and loss as probe_tip_localisation.py's crop_tuned variant, plus ring
shift augmentation: each frame is also used with the ring redrawn SHIFTS times at the target + a
random offset of up to SHIFT_MAX_PX (label shifted with it), so small offsets around the target -
the refinement's working range - are well covered, not just the recorded final frames.
HOLDOUT of the episodes (grouped, finish batch and older datasets alike) are held out and
reported; the saved model is then refitted on everything.

Writes outputs/tip_localiser/<time>/: model.pt (state_dict), config.json, summary.txt.
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
from torch import nn

sys.path.insert(0, str(Path(__file__).parent))
import probe_tip_localisation as ptl  # noqa: E402

from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.overlay import draw_target  # noqa: E402

DATASETS = ptl.DATASETS + ["basler_stage_touch_finish_2026-11"]
CROP_PX = 192
SHIFTS, SHIFT_MAX_PX = 3, 20.0
HOLDOUT, EPOCHS, BATCH, SEED = 0.1, 30, 32, 0
# Only frames where the tip is inside the crop can be localised; farther ones only teach "far".
MAX_OFFSET_PX = 80.0


def build(df: pd.DataFrame, style, shifts: int, rng) -> tuple[torch.Tensor, np.ndarray, np.ndarray]:
    """Crops (uint8 NCHW), labels (tip - ring centre, px) and the row index of each crop."""
    imgs, labels, rows = [], [], []
    for i, r in enumerate(df.itertuples()):
        raw = cv2.cvtColor(cv2.imread(r.png), cv2.COLOR_BGR2RGB)
        offsets = [np.zeros(2)] + [rng.uniform(-SHIFT_MAX_PX, SHIFT_MAX_PX, 2) for _ in range(shifts)]
        for o in offsets:
            c = np.array([r.target_u, r.target_v]) + o
            lab = np.array([r.tip_u, r.tip_v]) - c
            if np.abs(lab).max() > MAX_OFFSET_PX:
                continue
            imgs.append(ptl.crop(draw_target(raw, c, style), c[0], c[1]))
            labels.append(lab)
            rows.append(i)
    return ptl.to_tensor(imgs), np.asarray(labels), np.asarray(rows)


def fit(x: torch.Tensor, y: np.ndarray, epochs: int) -> nn.Module:
    torch.manual_seed(SEED)
    feat = ((CROP_PX + 31) // 32, (CROP_PX + 31) // 32)
    model = ptl.Tuned(feat).to(ptl.DEVICE)
    opt = torch.optim.AdamW(
        [{"params": model.head.parameters(), "lr": ptl.LR_HEAD}, {"params": model.backbone.parameters(), "lr": ptl.LR_BACKBONE}],
        weight_decay=ptl.WEIGHT_DECAY,
    )
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
    yt = torch.from_numpy(y / ptl.LABEL_SCALE_PX).float()
    rng = np.random.default_rng(SEED)
    for _ in range(epochs):
        model.train()
        for b in np.array_split(rng.permutation(len(y)), max(1, len(y) // BATCH)):
            loss = nn.functional.smooth_l1_loss(model(x[b].to(ptl.DEVICE)), yt[b].to(ptl.DEVICE), beta=0.05)
            opt.zero_grad()
            loss.backward()
            opt.step()
        sched.step()
    return model.eval()


@torch.inference_mode()
def predict(model: nn.Module, x: torch.Tensor) -> np.ndarray:
    out = [model(x[b].to(ptl.DEVICE)).float().cpu().numpy() for b in np.array_split(np.arange(len(x)), max(1, len(x) // BATCH))]
    return np.concatenate(out) * ptl.LABEL_SCALE_PX


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    ptl.DATASETS = DATASETS
    style = BenchConfig.load(ptl.RIG).target_style()
    um_per_px = BenchConfig.load(ptl.RIG).calibration().mm_per_px * 1000
    df = ptl.load_samples()
    rng = np.random.default_rng(SEED)
    epochs = 2 if args.quick else EPOCHS
    if args.quick:
        df = df.sample(200, random_state=SEED).reset_index(drop=True)
    groups = df.dataset + ":" + df.episode.astype(str)
    uniq = groups.unique()
    held = set(rng.choice(uniq, int(len(uniq) * HOLDOUT), replace=False))
    is_te = groups.isin(held).to_numpy()
    out = Path("outputs/tip_localiser") / time.strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=True)

    t0 = time.time()
    x_tr, y_tr, _ = build(df[~is_te], style, SHIFTS, rng)
    x_te, y_te, r_te = build(df[is_te], style, 0, rng)  # held-out: real ring positions only
    model = fit(x_tr, y_tr, epochs)
    err = np.linalg.norm(predict(model, x_te) - y_te, axis=1) * um_per_px
    zero = np.linalg.norm(y_te, axis=1) * um_per_px
    te = df[is_te].iloc[r_te].assign(err_um=err, zero_um=zero)
    lines = [f"{len(df)} frames from {len(uniq)} episodes; held out {len(held)} episodes ({len(te)} frames); "
             f"{len(y_tr)} training crops ({SHIFTS} ring shifts up to {SHIFT_MAX_PX} px); {epochs} epochs; {um_per_px:.2f} um/px"]
    for (ds, cp), g in te.groupby([te.dataset.str.replace("basler_stage_touch_", ""), "checkpoint"]):
        lines.append(f"{ds:20s} {cp:9s} n {len(g):4d}  median {g.err_um.median():5.0f} um  p90 {g.err_um.quantile(0.9):5.0f}  "
                     f"<=25 um {np.mean(g.err_um <= 25):4.0%}  (zero baseline {g.zero_um.median():5.0f})")
    lines.append(f"all held out: median {np.median(err):.0f} um, p90 {np.percentile(err, 90):.0f} um")

    # refit on everything for deployment
    x_all, y_all, _ = build(df, style, SHIFTS, rng)
    model = fit(torch.cat([x_all]), y_all, epochs)
    torch.save(model.state_dict(), out / "model.pt")
    (out / "config.json").write_text(json.dumps({
        "crop_px": CROP_PX, "label_scale_px": ptl.LABEL_SCALE_PX, "feat_hw": [(CROP_PX + 31) // 32] * 2,
        "datasets": DATASETS, "shifts": SHIFTS, "shift_max_px": SHIFT_MAX_PX, "epochs": epochs,
        "target_style": style.__dict__,
    }, indent=1, default=list))
    lines.append(f"refitted on all {len(y_all)} crops; {time.time() - t0:.0f} s")
    (out / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"-> {out}")


if __name__ == "__main__":
    main()
