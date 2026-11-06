"""Where along a rollout does the tip-to-target error arise? Needs no robot.

    python scripts/meca500/bench_error_timeline.py outputs/eval_auto/<run> [...] [--dataset NAME ...]

For each bench episode (forward kinematics on the measured joints) or recorded expert episode
(commanded joint targets), at four moments - the tip entering the frame, precision-flag entry,
closest approach after that entry, and the end - it reports the tip's XY error to the target split
along and across the pipette's XY direction (the tip frame's z axis projected on the dish), and
the aimed landing point: where the tip would land continuing along the pipette axis to the dish
surface. Moving along the axis changes only the along component, so the across error at precision
entry against the end says whether the final across-axis miss was already set.

For bench runs it also splits the path walked after precision entry by the precision flag (off
steps are clamped to 1 deg/step, on steps to 0.02 deg/step) and counts clamped steps.
"""

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import pandas as pd

from lerobot.robots.meca500.autocollect.bench import BenchConfig

RIG = "meca500_basler"
HOME = Path.home() / ".cache/huggingface/lerobot/AdamAxelrod"


def components(kin, calib, q, target_xy):
    """Per frame: tip XY error and aimed landing error, as (along, across) in mm, and tip pixels."""
    poses = [kin.tip(qq) for qq in q]
    p = np.array([t[:3, 3] for t in poses])
    a = np.array([t[:3, 2] for t in poses])  # pipette axis, out of the tip
    d = a[:, :2] / np.linalg.norm(a[:, :2], axis=1, keepdims=True)
    n = np.stack([-d[:, 1], d[:, 0]], axis=1)
    s = (calib.surface_z_mm - p[:, 2]) / a[:, 2]
    aim = p[:, :2] + s[:, None] * a[:, :2]
    e_tip, e_aim = p[:, :2] - target_xy, aim - target_xy
    split = lambda e: np.stack([(e * d).sum(1), (e * n).sum(1)], axis=1)
    px = np.asarray(calib.xy_to_pixel(p[:, :2]))
    return p, split(e_tip), split(e_aim), px


def moments(calib, p, prec, e_tip, px):
    w, h = calib.image_size
    inside = (px[:, 0] >= 0) & (px[:, 0] < w) & (px[:, 1] >= 0) & (px[:, 1] < h)
    out = {"enter": int(np.argmax(inside)) if inside.any() else None}
    if prec.any():
        ip = int(np.argmax(prec))
        out["prec"] = ip
        out["closest"] = ip + int(np.argmin(np.linalg.norm(e_tip[ip:], axis=1)))
    else:
        out["prec"] = out["closest"] = None
    out["end"] = len(p) - 1
    return out


def row(tag, ep, m, e_tip, e_aim):
    r = {"set": tag, "ep": ep}
    for k, i in m.items():
        for name, e in (("tip", e_tip), ("aim", e_aim)):
            r[f"{k}_{name}_along"] = np.nan if i is None else e[i, 0] * 1000
            r[f"{k}_{name}_across"] = np.nan if i is None else e[i, 1] * 1000
    return r


def bench_rows(run: Path, kin, calib):
    rows, flags = [], []
    for line in (run / "results.jsonl").read_text().splitlines():
        r = json.loads(line)
        f = run / f"ep{r['episode']:03d}" / "traj.npz"
        if not f.exists() or r["duration_s"] < 1:
            continue
        z = np.load(f)
        target = np.asarray(calib.pixel_to_xy(np.asarray(r["target_px"]))).ravel()[:2]
        p, e_tip, e_aim, px = components(kin, calib, z["q_meas"], target)
        prec = z["precision"].astype(bool)
        m = moments(calib, p, prec, e_tip, px)
        rows.append(row(run.name[:24], r["episode"], m, e_tip, e_aim))
        if m["prec"] is not None:
            pc = components(kin, calib, z["q_cmd"], target)[0]
            seg = np.linalg.norm(np.diff(pc[:, :2], axis=0), axis=1) * 1000
            ip, fl, cl = m["prec"], prec[:-1], z["clamped"][:-1].astype(bool)
            after = np.arange(len(seg)) >= ip
            flags.append({
                "set": run.name[:24], "ep": r["episode"],
                "path_flag_on_um": seg[after & fl].sum(), "path_flag_off_um": seg[after & ~fl].sum(),
                "steps_on": int((after & fl).sum()), "steps_off": int((after & ~fl).sum()),
                "clamped_after": int((after & cl).sum()), "clamped_total": int(z["clamped"].sum()),
                "toggles": int(np.abs(np.diff(prec.astype(int))).sum()),
            })
    return rows, flags


def dataset_rows(name: str, kin, calib):
    root = HOME / name
    meta = {
        (j := json.loads(line))["episode_index"]: j
        for line in (root / "autocollect_episodes.jsonl").read_text().splitlines()
        if line.strip()
    }
    df = pd.concat(
        pd.read_parquet(f, columns=["action", "episode_index", "frame_index"])
        for f in glob.glob(str(root / "data/*/*.parquet"))
    )
    rows = []
    for ep, g in df.groupby("episode_index"):
        a = np.stack(g.sort_values("frame_index").action.values)
        target = np.asarray(meta[ep]["target_xyz_mm"])[:2]
        p, e_tip, e_aim, px = components(kin, calib, a[:, :6], target)
        rows.append(row(f"expert {name}", ep, moments(calib, p, a[:, 6] > 0.5, e_tip, px), e_tip, e_aim))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", type=Path)
    ap.add_argument("--dataset", action="append", default=[], help="recorded (absolute) dataset name")
    ap.add_argument("--out", type=Path, help="per-episode CSV")
    args = ap.parse_args()
    bench = BenchConfig.load(RIG)
    kin, calib = bench.kinematics(), bench.calibration()
    rows, flags = [], []
    for run in args.runs:
        r, f = bench_rows(run, kin, calib)
        rows += r
        flags += f
    for name in args.dataset:
        rows += dataset_rows(name, kin, calib)
    d = pd.DataFrame(rows)
    pd.set_option("display.width", 250)
    for k in ("enter", "prec", "closest", "end"):
        cols = [f"{k}_tip_along", f"{k}_tip_across", f"{k}_aim_along"]
        print(f"\n{k}: median [IQR] um")
        g = d.groupby("set")[cols]
        print(g.median().round(0).astype("Int64").astype(str) + " [" + g.quantile(0.25).round(0).astype("Int64").astype(str) + ", " + g.quantile(0.75).round(0).astype("Int64").astype(str) + "]")
    print("\nacross-axis error: how much of the end value was there at precision entry")
    for s, g in d.groupby("set"):
        g = g.dropna(subset=["prec_tip_across"])
        ch = g.end_tip_across - g.prec_tip_across
        print(f"  {s:45s} n={len(g):3d}  |across| prec {g.prec_tip_across.abs().median():5.0f}  end {g.end_tip_across.abs().median():5.0f}"
              f"  change prec->end median {ch.median():+5.0f}, median |change| {ch.abs().median():5.0f} um"
              f"  corr(prec, end) {np.corrcoef(g.prec_tip_across, g.end_tip_across)[0, 1]:+.2f}")
    if flags:
        f = pd.DataFrame(flags)
        print("\nafter precision entry, path by flag state (commanded tip, um)")
        print(f.to_string(index=False))
        print(f.groupby("set")[["path_flag_on_um", "path_flag_off_um", "steps_on", "steps_off", "clamped_after", "toggles"]].median().round(0))
    if args.out:
        d.to_csv(args.out, index=False)


if __name__ == "__main__":
    main()
