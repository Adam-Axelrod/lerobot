"""Where does a policy's constant landing offset come from? Two offline checks. Needs a GPU, no robot.

    python scripts/meca500/probe_shift_causes.py --model <pretrained_model dir> [--dataset NAME]

A. CODEC. The policy is trained on AV1-decoded frames but sees raw camera frames live. For every
   EVERY-th usable episode, at start (frame 0, background.png), pretouch and near (as in
   probe_marker_shift.py), the same moment is fed twice - raw PNG + drawn marker, and the decoded
   training frame - with the same joint state. Reported: the SIGNED difference raw - AV1 in the
   predicted chunk's tip XY displacement (robot X/Y, mm), mean and SD.
B. INTEGRATED BIAS. Teacher-forced along each episode's coarse phase (precision flag off): every
   chunk_size frames, the recorded state and decoded frame go in, and the predicted chunk's tip XY
   displacement is compared with the recorded deltas' over the same frames from the same state.
   Reported: signed error per chunk, and summed over the coarse phase per episode (mean, SD).
A consistent sum near an observed landing offset points at that cause.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from probe_marker_shift import ROOT, RIG, Probe, hold_frames, load_states  # noqa: E402

from lerobot.datasets.lerobot_dataset import LeRobotDataset  # noqa: E402
from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.overlay import draw_target  # noqa: E402

EVERY = 5


def frame(ds, i) -> np.ndarray:
    item = ds[i]
    return (item["observation.images.microscope_cam"].permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True)
    ap.add_argument("--dataset", default="basler_stage_touch_dataset_2026-11")
    args = ap.parse_args()

    raw_root, delta_name = ROOT / args.dataset, f"{args.dataset}_delta"
    excl_file = raw_root / "autocollect/excluded_episodes.json"
    excluded = set(json.loads(excl_file.read_text())["excluded"]) if excl_file.exists() else set()
    bench = BenchConfig.load(RIG)
    kin, style = bench.kinematics(), bench.target_style()
    meta = {
        json.loads(line)["episode_index"]: json.loads(line)
        for line in (raw_root / "autocollect_episodes.jsonl").read_text().splitlines()
        if line.strip()
    }
    states = load_states(ROOT / delta_name)
    ds = LeRobotDataset(f"AdamAxelrod/{delta_name}", video_backend="pyav")
    probe = Probe(args.model)
    chunk = probe.policy.config.chunk_size
    eps = [e for e in sorted(meta) if e not in excluded and e in states][::EVERY]

    def disp(q, deltas) -> np.ndarray:
        return kin.tip(q + deltas[:, :6].sum(0))[:3, 3][:2] - kin.tip(q)[:3, 3][:2]

    codec = {"start": [], "pretouch": [], "near": []}
    per_chunk, sums, n_chunks = [], [], []
    for ep in eps:
        g = states[ep]
        q_all = np.stack(g["observation.state"].values)[:, :6].astype(float)
        act = np.stack(g["action"].values)
        i0 = int(ds.meta.episodes["dataset_from_index"][ep])
        holds = hold_frames(g)
        cps = {"start": (0, "background.png")}
        if holds:
            cps["pretouch"], cps["near"] = (holds[0], "pretouch.png"), (holds[-1], "near.png")
        for name, (fi, png) in cps.items():
            raw = cv2.cvtColor(cv2.imread(str(raw_root / "autocollect/raw" / f"ep{ep:04d}" / png)), cv2.COLOR_BGR2RGB)
            q = q_all[fi]
            d_raw = disp(q, probe.chunk(q, draw_target(raw, meta[ep]["target_px"], style)))
            d_av1 = disp(q, probe.chunk(q, frame(ds, i0 + fi)))
            codec[name].append(d_raw - d_av1)
        coarse_end = int(np.argmax(act[:, 6] > 0.5)) if (act[:, 6] > 0.5).any() else len(act)
        total = np.zeros(2)
        for s in range(0, coarse_end, chunk):
            e = min(s + chunk, coarse_end)
            pred = probe.chunk(q_all[s], frame(ds, i0 + s))[: e - s]
            err = disp(q_all[s], pred) - disp(q_all[s], act[s:e])
            per_chunk.append(err)
            total += err
        sums.append(total)
        n_chunks.append(int(np.ceil(coarse_end / chunk)))
        print(f"ep {ep:3d}: coarse {coarse_end} frames, bias sum {np.round(total, 3).tolist()} mm", flush=True)

    def ms(a) -> str:
        a = np.asarray(a)
        return f"X {a[:, 0].mean():+.3f} (SD {a[:, 0].std(ddof=1):.3f})  Y {a[:, 1].mean():+.3f} (SD {a[:, 1].std(ddof=1):.3f}) mm, n={len(a)}"

    print(f"\n{args.model}\n{len(eps)} episodes of {args.dataset}, chunk {chunk}")
    print("A. raw - AV1, predicted chunk tip XY displacement:")
    for k, v in codec.items():
        if v:
            print(f"   {k:8s} {ms(v)}")
    print(f"   coarse phase: {np.median(n_chunks):.0f} chunks (median)")
    print("B. predicted - recorded, teacher-forced coarse phase:")
    print(f"   per chunk   {ms(per_chunk)}")
    print(f"   per episode {ms(sums)}")


if __name__ == "__main__":
    main()
