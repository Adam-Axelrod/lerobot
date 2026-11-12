"""Re-score eval runs with "the tip" = the centre of the pipette opening (2026-11-12).

    python scripts/meca500/rescore_opening.py [RUN_GLOB ...] [--threshold-um 50]

Before 2026-11-12, eval_auto's vision_tip_px / vision_error_um were the end of the glass (detect_tip's
point). Adam's blind by-eye verdicts followed the pipette opening instead (THEORIES.md, "Where we
are"). For every episode with epNNN/final.png and background.png this re-detects the tip, finds the
opening (tip_detect.find_opening) and prints, per run, the opening error next to the glass-end error.
Writes <run>/results_opening.jsonl. Also prints the median opening - glass-end offset over all
frames, the number bench.yaml's target.opening_offset_px holds.
"""

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np

from lerobot.robots.meca500.autocollect.tip_detect import detect_tip, find_opening

UM_PER_PX = 13.40


def rgb(p: Path) -> np.ndarray | None:
    im = cv2.imread(str(p))
    return None if im is None else cv2.cvtColor(im, cv2.COLOR_BGR2RGB)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="*", default=["outputs/eval_auto/finish_2026-11_*"])
    ap.add_argument("--threshold-um", type=float, default=50.0)
    args = ap.parse_args()
    offsets = []
    for run in sorted({r for g in args.runs for r in glob.glob(g)}):
        run = Path(run)
        if not (run / "results.jsonl").exists():
            continue
        rows = []
        for line in (run / "results.jsonl").read_text().splitlines():
            r = json.loads(line)
            ep = run / f"ep{r['episode']:03d}"
            fr, bg = rgb(ep / "final.png"), rgb(ep / "background.png")
            det = None if fr is None or bg is None else detect_tip(fr, bg)
            if det is None:
                continue
            op, seen = find_opening(fr, det)
            t = np.asarray(r["target_px"], float)
            offsets.append(np.subtract(op, det.px))
            rows.append({
                "episode": r["episode"], "opening_px": [round(op[0], 2), round(op[1], 2)], "opening_seen": seen,
                "opening_error_um": round(float(np.linalg.norm(np.asarray(op) - t)) * UM_PER_PX, 1),
                "apex_px": [round(det.px[0], 2), round(det.px[1], 2)],
                "apex_error_um": round(float(np.linalg.norm(np.asarray(det.px) - t)) * UM_PER_PX, 1),
            })
        if not rows:
            continue
        with (run / "results_opening.jsonl").open("w", encoding="utf-8", newline="\n") as f:
            f.writelines(json.dumps(x) + "\n" for x in rows)
        o = np.array([x["opening_error_um"] for x in rows])
        a = np.array([x["apex_error_um"] for x in rows])
        print(f"{run.name}\n  n {len(rows):2d} | opening: median {np.median(o):6.0f} um, <= {args.threshold_um:.0f} um in {np.sum(o <= args.threshold_um):2d}"
              f" | glass end: median {np.median(a):6.0f} um, <= {args.threshold_um:.0f} um in {np.sum(a <= args.threshold_um):2d}")
    if offsets:
        v = np.array(offsets)
        print(f"\nopening - glass end over {len(v)} frames: median {np.median(v, 0).round(2).tolist()} px")


if __name__ == "__main__":
    main()
