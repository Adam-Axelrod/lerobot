"""Does a microscope policy aim by its joints or by the image? Slides the robot base in software on
real recorded frames and measures how the predicted action chunk responds. Needs a GPU, no robot.

    python scripts/meca500/probe_slide.py --model <pretrained_model dir> [--dataset NAME]

The same frames and checkpoints as probe_marker_shift.py (start, pretouch, near). The image and the
marker stay as recorded; the joint state is replaced by the virtual joints of a base slid by SLIDE_MM
along +x, -x, +y and -y (autocollect/asr_bridge.VirtualSlide, as eval_auto's software slide).
    follow   (change in the chunk's tip XY displacement) . d / |d|^2, averaged over the 4 slides.
             1 = the chunk moves the tip by the slide (it aims by the joints, and lands d off);
             0 = it ignores the slide (it aims by what the image shows). At "start" the tip is out
             of view, so the joints are all the policy has; pretouch and near are what matter.
    resp_mm  |change in displacement|, any direction.
The medians per checkpoint are printed; per-row results go to --out.
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from probe_marker_shift import ROOT, RIG, EVERY, Probe, hold_frames, load_states  # noqa: I001

import cv2

from lerobot.robots.meca500.autocollect.asr_bridge import VirtualSlide
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.overlay import draw_target

SLIDE_MM = 2.0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help="pretrained_model directory")
    ap.add_argument("--dataset", default="basler_stage_touch_decoy_2026-11", help="recorded (absolute) dataset name")
    ap.add_argument("--delta-dataset", default=None, help="default: <dataset>_delta")
    ap.add_argument("--every", type=int, default=EVERY, help="probe every Nth usable episode")
    ap.add_argument("--out", default=None, help="per-row CSV (default: <model>/../probe_slide.csv)")
    args = ap.parse_args()

    raw_root = ROOT / args.dataset
    delta_root = ROOT / (args.delta_dataset or f"{args.dataset}_delta")
    raw = raw_root / "autocollect/raw"
    excl_file = raw_root / "autocollect/excluded_episodes.json"
    excluded = set(json.loads(excl_file.read_text())["excluded"]) if excl_file.exists() else set()

    bench = BenchConfig.load(RIG)
    kin, style = bench.kinematics(), bench.target_style()
    meta = {
        json.loads(line)["episode_index"]: json.loads(line)
        for line in (raw_root / "autocollect_episodes.jsonl").read_text().splitlines()
        if line.strip()
    }
    states = load_states(delta_root)
    probe = Probe(args.model)
    eps = [e for e in sorted(meta) if e not in excluded and e in states][:: args.every]
    slides = [np.array(d) * SLIDE_MM for d in ((1, 0), (-1, 0), (0, 1), (0, -1))]

    def disp(q, deltas) -> np.ndarray:
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
            img = draw_target(cv2.cvtColor(cv2.imread(str(raw / f"ep{ep:04d}" / png)), cv2.COLOR_BGR2RGB), target, style)
            q = np.stack(g["observation.state"].values)[fi][:6].astype(float)
            base = disp(q, probe.chunk(q, img))
            resp = []
            for d in slides:
                qv = VirtualSlide(kin, d).to_virtual(q)
                if qv is None:
                    continue
                dd = disp(qv, probe.chunk(qv, img)) - base
                resp.append((float(dd @ d / (d @ d)), float(np.linalg.norm(dd))))
            if resp:
                rows.append({
                    "ep": ep, "cp": name, "chunk_mm": float(np.linalg.norm(base)),
                    "follow": float(np.mean([r[0] for r in resp])), "resp_mm": float(np.mean([r[1] for r in resp])),
                })

    df = pd.DataFrame(rows)
    out = Path(args.out) if args.out else Path(args.model).parent / "probe_slide.csv"
    df.to_csv(out, index=False)
    print(f"{args.model}\n{len(eps)} episodes of {args.dataset}; base slid {SLIDE_MM} mm")
    print(df.groupby("cp")[["chunk_mm", "follow", "resp_mm"]].median().round(3).to_string())
    print(f"rows: {out}")


if __name__ == "__main__":
    main()
