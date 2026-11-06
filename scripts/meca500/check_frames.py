"""Scale marks on an eval_auto run's final frames, for the by-eye check. Needs no robot.

    python scripts/meca500/check_frames.py outputs/eval_auto/<run>

Writes <run>/check/epNNN.png and <run>/check/all.png (every episode). Each image is centred on the
target and zoomed as far as the detected tip allows, so the scale differs between images: go by
the circles and the bar. With no detected tip it shows a wide view. It shows:
    green circle   the 50 um success zone
    yellow circle  0.6 mm
    orange circle  1 mm
    white bar      1 mm
    red cross      where the tip detector put the tip, if it found one
The target ring drawn for the policy has a radius of about 180 um, so a tip can be in the ring
and still outside the success zone.

Results go in <run>/verdicts.json, read by score_eval.py; see its docstring.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from lerobot.robots.meca500.autocollect.bench import BenchConfig

RIG = "meca500_basler"
OUT_SIZE = 440
MIN_HALF_WINDOW_PX = 110  # about 1.4 mm each side of the target
WIDE_HALF_WINDOW_PX = 350  # about 4.5 mm, when there is no detected tip to fit in
TIP_MARGIN_PX = 40


def half_window(target, tip) -> int:
    if tip is None:
        return WIDE_HALF_WINDOW_PX
    return max(
        MIN_HALF_WINDOW_PX, round(float(np.hypot(tip[0] - target[0], tip[1] - target[1]))) + TIP_MARGIN_PX
    )


def annotate(frame: np.ndarray, target, um_per_px: float, tip=None, label: str = "") -> np.ndarray:
    """``frame`` (BGR) cropped round the target and resized, with the scale marks drawn on."""
    u, v = float(target[0]), float(target[1])
    half = half_window(target, tip)
    x0, y0 = round(u) - half, round(v) - half
    size = 2 * half
    crop = np.zeros((size, size, 3), np.uint8)  # black where the window leaves the image
    h, w = frame.shape[:2]
    xa, ya, xb, yb = max(x0, 0), max(y0, 0), min(x0 + size, w), min(y0 + size, h)
    if xb > xa and yb > ya:
        crop[ya - y0 : yb - y0, xa - x0 : xb - x0] = frame[ya:yb, xa:xb]
    zoom = OUT_SIZE / size
    interpolation = cv2.INTER_NEAREST if zoom >= 1 else cv2.INTER_AREA
    out = cv2.resize(crop, (OUT_SIZE, OUT_SIZE), interpolation=interpolation)

    def to_view(px, py):
        return round((px - x0 + 0.5) * zoom), round((py - y0 + 0.5) * zoom)

    centre = to_view(u, v)
    for radius_um, colour in ((50, (0, 255, 0)), (600, (0, 255, 255)), (1000, (0, 140, 255))):
        cv2.circle(out, centre, max(round(radius_um / um_per_px * zoom), 2), colour, 1, cv2.LINE_AA)
    if tip is not None:
        x, y = to_view(tip[0], tip[1])
        cv2.drawMarker(out, (x, y), (0, 0, 255), cv2.MARKER_CROSS, 13, 1, cv2.LINE_AA)
    bar = round(1000 / um_per_px * zoom)
    y = out.shape[0] - 12
    cv2.line(out, (10, y), (10 + bar, y), (0, 0, 0), 5)
    cv2.line(out, (10, y), (10 + bar, y), (255, 255, 255), 3)
    cv2.putText(out, "1 mm", (10, y - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(out, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(out, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def trial_label(row: dict) -> str:
    error = row.get("vision_error_um")
    distance = "not detected" if error is None else f"{error:.0f} um"
    ending = str(row.get("reason", "")).split(":")[0]
    return f"ep {row['episode']}  detector {distance}  {ending}"


def contact_sheet(images: list[np.ndarray], columns: int = 5) -> np.ndarray:
    tile = images[0].shape
    rows = [images[i : i + columns] for i in range(0, len(images), columns)]
    blank = np.zeros(tile, np.uint8)
    return np.vstack([np.hstack(row + [blank] * (columns - len(row))) for row in rows])


def check_dataset(args, um_per_px: float) -> None:
    """The recorder's own final frames, marked as for a run. No verdicts are drawn."""
    lines = (args.dataset / "autocollect_episodes.jsonl").read_text(encoding="utf-8").splitlines()
    rows = {r["episode_index"]: r for r in (json.loads(line) for line in lines if line.strip())}
    out_dir = args.out or args.dataset / "autocollect" / "check"
    out_dir.mkdir(parents=True, exist_ok=True)
    images = []
    for ep in args.episodes or sorted(rows):
        row = rows[ep]
        frame = cv2.imread(str(args.dataset / "autocollect" / "raw" / f"ep{ep:04d}" / "final.png"))
        if frame is None:
            print(f"Missing final frame for episode {ep}: skipped.")
            continue
        final = row.get("final") or {}
        tip = final.get("detected_px")
        err = row.get("final_error_um")
        label = f"ep {ep}  detector: " + ("no tip" if tip is None or err is None else f"{err:.0f} um")
        image = annotate(frame, row["target_px"], um_per_px, tip, label)
        cv2.imwrite(str(out_dir / f"ep{ep:04d}.png"), image)
        images.append(image)
    if images:
        cv2.imwrite(str(out_dir / "all.png"), contact_sheet(images))
    print(f"{len(images)} images at {um_per_px:.2f} um/px -> {out_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("run", type=Path, nargs="?", help="an outputs/eval_auto/<run> folder")
    parser.add_argument(
        "--dataset", type=Path, help="a record_auto dataset root instead: its autocollect/raw/epNNNN/final.png"
    )
    parser.add_argument("--episodes", type=int, nargs="+", help="--dataset: only these episodes")
    parser.add_argument("--out", type=Path, help="--dataset: output folder (default <dataset>/autocollect/check)")
    args = parser.parse_args()
    um_per_px = BenchConfig.load(RIG).calibration().mm_per_px * 1000
    if args.dataset is not None:
        check_dataset(args, um_per_px)
        return
    if args.run is None:
        parser.error("give a run folder, or --dataset")
    lines = (args.run / "results.jsonl").read_text(encoding="utf-8").splitlines()
    rows = [json.loads(line) for line in lines if line.strip()]
    out_dir = args.run / "check"
    out_dir.mkdir(exist_ok=True)
    images = []
    for row in rows:
        path = args.run / f"ep{row['episode']:03d}" / "final.png"
        frame = cv2.imread(str(path))
        if frame is None:
            print(f"Missing {path}: skipped.")
            continue
        image = annotate(frame, row["target_px"], um_per_px, row.get("vision_tip_px"), trial_label(row))
        cv2.imwrite(str(out_dir / f"ep{row['episode']:03d}.png"), image)
        images.append(image)
    if images:
        cv2.imwrite(str(out_dir / "all.png"), contact_sheet(images))
    print(f"{len(images)} images at {um_per_px:.2f} um/px -> {out_dir}")


if __name__ == "__main__":
    main()
