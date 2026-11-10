"""E2 ep 87 of basler_stage_touch_dataset_2026-11: final frame, target ring, our detector's tip point."""

import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

D = Path(os.path.expanduser("~/.cache/huggingface/lerobot/AdamAxelrod/basler_stage_touch_dataset_2026-11"))
EP = 87
RING_R, HALF, ZOOM = 14, 40, 4
C_TGT, C_DET = (60, 220, 60), (235, 200, 0)
OUT = Path(sys.argv[1])

rec = next(json.loads(l) for l in open(D / "autocollect_episodes.jsonl") if json.loads(l)["episode_index"] == EP)
excl = json.load(open(D / "autocollect/excluded_episodes.json"))["excluded"]
img = cv2.imread(str(D / f"autocollect/raw/ep{EP:04d}/final.png"))
target, det = np.array(rec["target_px"]), np.array(rec["final"]["detected_px"])


def marks(im, scale, off):
    to = lambda p: (int(round((p[0] - off[0] + 0.5) * scale)), int(round((p[1] - off[1] + 0.5) * scale)))  # noqa: E731
    t = to(target)
    cv2.circle(im, t, int(RING_R * scale), (0, 0, 0), 4, cv2.LINE_AA)
    cv2.circle(im, t, int(RING_R * scale), C_TGT, 2, cv2.LINE_AA)
    for s, c in ((7, C_TGT),):
        cv2.line(im, (t[0] - s, t[1]), (t[0] + s, t[1]), c, 1, cv2.LINE_AA)
        cv2.line(im, (t[0], t[1] - s), (t[0], t[1] + s), c, 1, cv2.LINE_AA)
    d = to(det)
    for a, b in (((-9, -9), (9, 9)), ((-9, 9), (9, -9))):
        cv2.line(im, (d[0] + a[0], d[1] + a[1]), (d[0] + b[0], d[1] + b[1]), (0, 0, 0), 4, cv2.LINE_AA)
        cv2.line(im, (d[0] + a[0], d[1] + a[1]), (d[0] + b[0], d[1] + b[1]), C_DET, 2, cv2.LINE_AA)
    return im


def crop(centre):
    cx, cy = int(round(centre[0])), int(round(centre[1]))
    pad = cv2.copyMakeBorder(img, HALF, HALF, HALF, HALF, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    c = cv2.resize(pad[cy : cy + 2 * HALF, cx : cx + 2 * HALF], None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
    return marks(c, ZOOM, (cx - HALF, cy - HALF))


full = marks(img.copy(), 1, (0, 0))
full = cv2.resize(full, (full.shape[1] * 320 // full.shape[0] * 2, 640))  # unchanged size: 427x640
c1, c2 = crop(target), crop(det)
lab = lambda im, s: cv2.putText(im, s, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)  # noqa: E731
lab(c1, "4x around the target ring")
lab(c2, "4x around the detected tip")
right = np.vstack([c1, np.full((6, c1.shape[1], 3), 255, np.uint8), c2])
right = right[:640] if right.shape[0] >= 640 else np.vstack([right, np.full((640 - right.shape[0], right.shape[1], 3), 255, np.uint8)])
body = np.hstack([full, np.full((640, 6, 3), 255, np.uint8), right])

lines = [
    f"basler_stage_touch_dataset_2026-11, E2 ep {EP} (seed {rec['seed']}): raw/ep{EP:04d}/final.png, full res 427x640",
    f"green ring = target {rec['target_px']} (r {RING_R} px); cyan x = our detector's final tip point {rec['final']['detected_px']}",
    f"logged final_error_um = {rec['final_error_um']}   in excluded_episodes.json: {'YES' if EP in excl else 'NO (in the training data)'}",
]
for c in rec["corrections"] + [dict(rec["final"], checkpoint="final")]:
    lines.append(f"  {c['checkpoint']:8s} detected {c['detected_px']}  error {c['error_px']} px  shift {c['shift_um']} um  {c['note'] or '(applied)'}")
hdr = np.full((22 + 19 * len(lines), body.shape[1], 3), 255, np.uint8)
for i, s in enumerate(lines):
    cv2.putText(hdr, s, (8, 20 + 19 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 1, cv2.LINE_AA)
cv2.imwrite(str(OUT), np.vstack([hdr, body]), [cv2.IMWRITE_PNG_COMPRESSION, 9])
print(OUT.stat().st_size, "excluded:", EP in excl)
