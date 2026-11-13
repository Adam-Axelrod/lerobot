"""Collect frames with the opening detector's point for the by-eye check -> JSON for the review page.

    python scripts/meca500/build_opening_review.py OUT.json

Each item is a crop centred on the detected opening (tip_detect.find_opening) with the opening and
the glass end (detect_tip's point) in crop pixels, from a mix of frames: the finish batch's final,
near-hold and pre-touch frames, the 12 Nov bench final frames, and every frame where the opening
was not seen (the fixed fallback). Shuffled, with neutral ids; the key is written next to OUT.
"""

import base64
import glob
import json
import os
import random
import sys

import cv2
import numpy as np

from lerobot.robots.meca500.autocollect.tip_detect import detect_tip, find_opening

HALF = 40  # crop +-40 px = +-0.54 mm
FIN = os.path.expanduser("~/.cache/huggingface/lerobot/AdamAxelrod/basler_stage_touch_finish_2026-11/autocollect/raw/")
BENCH = "outputs/eval_auto/finish_2026-11_crop256_*20261112-*"


def rgb(p):
    im = cv2.imread(p)
    return None if im is None else cv2.cvtColor(im, cv2.COLOR_BGR2RGB)


def item(src, kind, frame_p, bg_p):
    fr, bg = rgb(frame_p), rgb(bg_p)
    if fr is None or bg is None:
        return None
    det = detect_tip(fr, bg)
    if det is None or det.score < 0.15:
        return None
    op, seen = find_opening(fr, det)
    c = np.round(op).astype(int)
    pad = cv2.copyMakeBorder(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR), HALF, HALF, HALF, HALF, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    crop = pad[c[1]: c[1] + 2 * HALF, c[0]: c[0] + 2 * HALF]
    ok, buf = cv2.imencode(".png", crop)
    to_crop = lambda p: [round(float(p[0] - c[0] + HALF), 2), round(float(p[1] - c[1] + HALF), 2)]
    return {"src": src, "kind": kind, "seen": bool(seen), "img": "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode(),
            "opening": to_crop(op), "apex": to_crop(det.px), "offset_px": round(float(np.hypot(*np.subtract(op, det.px))), 2),
            "score": round(det.score, 2)}


def main():
    out = sys.argv[1]
    rng = random.Random(11)
    eps = sorted(glob.glob(FIN + "ep*"))
    cands = []
    for d in eps:
        for kind in ("final", "near", "pretouch"):
            cands.append((f"finish_dataset:{os.path.basename(d)}:{kind}", kind, f"{d}/{kind}.png", f"{d}/background.png"))
    for d in sorted(glob.glob(BENCH + "/ep*")):
        cands.append((f"{os.path.basename(os.path.dirname(d))}:{os.path.basename(d)}", "final (bench)", f"{d}/final.png", f"{d}/background.png"))
    items, fallbacks = [], []
    for c in cands:
        it = item(*c)
        if it is None:
            continue
        (items if it["seen"] else fallbacks).append(it)
    by_kind = {}
    for it in items:
        by_kind.setdefault(it["kind"], []).append(it)
    want = {"final": 16, "near": 14, "pretouch": 10, "final (bench)": 14}
    chosen = rng.sample(fallbacks, min(20, len(fallbacks)))  # all of them is too many to judge
    for k, n in want.items():
        pool = by_kind.get(k, [])
        chosen += rng.sample(pool, min(n, len(pool)))
    rng.shuffle(chosen)
    for i, it in enumerate(chosen):
        it["id"] = f"T{i + 1:02d}"
    json.dump({"half_px": HALF, "um_per_px": 13.4, "items": chosen}, open(out, "w"))
    key = [{k: v for k, v in it.items() if k != "img"} for it in chosen]
    os.makedirs("outputs/finish_review", exist_ok=True)
    json.dump(key, open("outputs/finish_review/opening_review_key_20261113.json", "w"), indent=1)
    from collections import Counter
    print(len(chosen), "items:", dict(Counter(i["kind"] for i in chosen)), "| fallbacks", len(fallbacks),
          "| of", len(items) + len(fallbacks), "detections")


if __name__ == "__main__":
    main()
