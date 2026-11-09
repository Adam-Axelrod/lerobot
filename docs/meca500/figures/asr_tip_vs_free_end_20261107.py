"""Contact sheet: ASR's tip point vs our free-end detection, 7 Nov policy_asr x20 run (0 mm).

Per trial, two crops at 4x around the tip:
  A. the servo-height frame (last episode.mp4 frame before the touch stage = where eval_auto grabbed
     the frame for servo.free_end_px). Markers are the LOGGED points from results.jsonl. The video
     keeps every frame at half resolution (f[::2, ::2]) upsampled to 428x640, so this crop is soft.
  B. final.png (full resolution, after the 0.15 mm touch), with BOTH detectors RE-RUN on it here.
Also re-runs our detector on frame A as a timing/resolution check against the logged free_end_px.
"""

import json
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

REPO = Path(r"C:\Users\aa24\PhD\lerobot")
sys.path.insert(0, str(REPO / "src"))
from lerobot.robots.meca500.autocollect.asr_bridge import AsrTipDetector  # noqa: E402
from lerobot.robots.meca500.autocollect.tip_detect import detect_tip  # noqa: E402

RUN = REPO / "outputs/eval_auto/basler_stage_touch_decoyrec_2026-11_delta_act_asr_20261107-143233"
TRIALS = [6, 11, 19, 2, 9, 3, 10]
FLAGGED = {6, 11, 19}
OFFSET = np.array([-0.6, 22.92])  # calibrated ASR - free end, 7 Nov
HALF = 48  # crop half-size in original px (96 px = 1.28 mm)
ZOOM = 4
RING_R = 14  # overlay ring radius px (bench.yaml target.style)
UM_PER_PX = 13.3
OUT = Path(sys.argv[1])

# BGR colours
C_ASR, C_FREE, C_TGT, C_PRED = (40, 40, 235), (235, 200, 0), (60, 220, 60), (0, 210, 255)


def cross(img, p, c, s=9, t=2, diag=False):
    x, y = int(round(p[0])), int(round(p[1]))
    for (dx1, dy1, dx2, dy2) in ([(-s, -s, s, s), (-s, s, s, -s)] if diag else [(-s, 0, s, 0), (0, -s, 0, s)]):
        cv2.line(img, (x + dx1, y + dy1), (x + dx2, y + dy2), (0, 0, 0), t + 2, cv2.LINE_AA)
        cv2.line(img, (x + dx1, y + dy1), (x + dx2, y + dy2), c, t, cv2.LINE_AA)


def crop(rgb, centre, marks, ring_c):
    """marks: [(px, colour, diag)] in original px. Returns a BGR crop at ZOOM."""
    cx, cy = int(round(centre[0])), int(round(centre[1]))
    pad = cv2.copyMakeBorder(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), HALF, HALF, HALF, HALF, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    c = pad[cy : cy + 2 * HALF, cx : cx + 2 * HALF]
    big = cv2.resize(c, None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
    to = lambda p: ((p[0] - cx + HALF + 0.5) * ZOOM, (p[1] - cy + HALF + 0.5) * ZOOM)  # noqa: E731
    tc = to(ring_c)
    cv2.circle(big, (int(tc[0]), int(tc[1])), RING_R * ZOOM, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.circle(big, (int(tc[0]), int(tc[1])), RING_R * ZOOM, C_TGT, 2, cv2.LINE_AA)
    cross(big, tc, C_TGT, s=7, t=1)
    for p, col, diag in marks:
        if p is not None:
            cross(big, to(p), col, diag=diag)
    return big


def text(img, lines, x=6, y=18, dy=19, scale=0.45):
    """Caption in a white strip above the crop (returns the stacked image)."""
    strip = np.full((y + dy * len(lines) - 8, img.shape[1], 3), 255, np.uint8)
    for i, s in enumerate(lines):
        cv2.putText(strip, s, (x, y + i * dy), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 1, cv2.LINE_AA)
    return np.vstack([strip, img])


def main():
    from action_state_runtime.config.runtime import load_config
    from action_state_runtime.runtime import ActionStateRuntime
    import yaml

    bench = yaml.safe_load((REPO / "assets/benches/meca500_basler/bench.yaml").read_text())
    tc = bench["target"]
    runtime = ActionStateRuntime(config=load_config(str(REPO / "assets/benches/meca500_basler/asr_site.yaml")), backend_mode="real", audit_dir=tempfile.mkdtemp())
    runtime.preload_detector()
    asr = AsrTipDetector(runtime, tc["field_centre_px"], tc["field_radius_px"])

    res = {json.loads(l)["episode"]: json.loads(l) for l in open(RUN / "results.jsonl")}
    rows, report = [], []
    for ep in TRIALS:
        r = res[ep]
        s = r["servo"]
        d = RUN / f"ep{ep:03d}"
        target = np.array(r["target_px"])
        st = {x["stage"]: x for x in r["stages"]}
        t_end = st["servo"]["t_s"] + st["servo"]["duration_s"]
        ts = json.load(open(d / "episode_t.json"))
        k = max(i for i, t in enumerate(ts) if t <= t_end)
        cap = cv2.VideoCapture(str(d / "episode.mp4"))
        cap.set(cv2.CAP_PROP_POS_FRAMES, k)
        ok, vbgr = cap.read()
        assert ok
        # video px -> original px: original ~= video - 0.5 (2x linear upsample of f[::2, ::2]); 428 -> 427 wide
        M = np.float32([[1, 0, -0.5], [0, 1, -0.5]])
        vrgb = cv2.cvtColor(cv2.warpAffine(vbgr, M, (427, 640), flags=cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)
        bg = cv2.cvtColor(cv2.imread(str(d / "background.png")), cv2.COLOR_BGR2RGB)
        final = cv2.cvtColor(cv2.imread(str(d / "final.png")), cv2.COLOR_BGR2RGB)

        asr_log, free_log = np.array(s["final_tip_px"]), np.array(s["free_end_px"])
        pred = free_log + OFFSET
        free_rerun_a = detect_tip(vrgb, bg)
        asr_rerun_a = asr.detect(vrgb)
        free_b = detect_tip(final, bg)
        asr_b = asr.detect(final)
        fb = None if free_b is None else np.array(free_b.px)
        ab = None if asr_b is None else np.array(asr_b.px)

        centre = (asr_log + free_log) / 2
        A = crop(vrgb, centre, [(pred, C_PRED, True), (free_log, C_FREE, True), (asr_log, C_ASR, False)], target)
        Bc = (ab + fb) / 2 if (ab is not None and fb is not None) else centre
        B = crop(final, Bc, [(fb, C_FREE, True), (ab, C_ASR, False)], target)
        dA = asr_log - free_log
        tag = "FLAGGED" if ep in FLAGGED else "good"
        A = text(A, [f"ep{ep:02d} {tag}  A: servo height (video, t={ts[k]:.1f}s)",
                 f"logged ASR-free ({dA[0]:+.1f},{dA[1]:+.1f}) px  [cal (-0.6,+22.9)]",
                 f"drift from cal {np.linalg.norm(dA - OFFSET):.1f} px",
                 f"logged free-end residual {s['free_end_to_target_um']:.0f} um"])
        dB = None if (ab is None or fb is None) else ab - fb
        B = text(B, [f"ep{ep:02d}  B: final.png full-res, both RE-RUN",
                 "ASR-free " + ("n/a" if dB is None else f"({dB[0]:+.1f},{dB[1]:+.1f}) px"),
                 "after the vertical 0.15 mm touch",
                 f"final distance (ours) {r.get('vision_error_um', float('nan')):.0f} um"])
        sep = np.full((A.shape[0], 6, 3), 255, np.uint8)
        rows.append(np.hstack([A, sep, B]))
        report.append({
            "ep": ep, "video_frame": k, "t": ts[k], "servo_end_t": round(t_end, 2),
            "logged_asr": asr_log.tolist(), "logged_free": free_log.tolist(),
            "rerun_free_on_A": None if free_rerun_a is None else np.round(free_rerun_a.px, 2).tolist(),
            "rerun_asr_on_A": None if asr_rerun_a is None else [round(v, 2) for v in asr_rerun_a.px],
            "rerun_free_on_final": None if fb is None else np.round(fb, 2).tolist(),
            "rerun_asr_on_final": None if ab is None else np.round(ab, 2).tolist(),
            "logged_vision_tip_final": r.get("vision_tip_px"),
        })

    W = rows[0].shape[1]
    hdr = np.full((170, W, 3), 255, np.uint8)
    cv2.putText(hdr, "ASR tip point vs our free-end detection, policy_asr x20, 7 Nov (0 mm), 4x crops (96 px = 1.28 mm)", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
    leg = [(C_ASR, False, "ASR tip point (A: logged servo.final_tip_px; B: re-run)"),
           (C_FREE, True, "our free-end detector (A: logged servo.free_end_px; B: re-run)"),
           (C_PRED, True, "A only: free end + calibrated offset (-0.6, +22.9) = where ASR's point 'should' be"),
           (C_TGT, False, "target ring (r 14 px) and centre"),
           (None, False, "A: video, half-res + mp4, eval_auto's ring burned in (thick). B: full-res, no overlay.")]
    for i, (c, diag, s) in enumerate(leg):
        if c is not None:
            cross(hdr, (20, 50 + i * 24), c, s=8, diag=diag)
        cv2.putText(hdr, s, (40, 55 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    sheet = np.vstack([hdr] + [np.vstack([r_, np.full((6, W, 3), 255, np.uint8)]) for r_ in rows])
    cv2.imwrite(str(OUT), sheet, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    for q in report:
        print(q["ep"], "t", q["t"], "logged ASR-free", [round(a-b,1) for a,b in zip(q["logged_asr"],q["logged_free"])], "| final re-run ASR", q["rerun_asr_on_final"], "free", q["rerun_free_on_final"], "ASR-free", None if None in (q["rerun_asr_on_final"],q["rerun_free_on_final"]) else [round(a-b,1) for a,b in zip(q["rerun_asr_on_final"],q["rerun_free_on_final"])], "| free final vs logged servo", [round(a-b,1) for a,b in zip(q["rerun_free_on_final"],q["logged_free"])])
    print("size", OUT.stat().st_size, sheet.shape)


if __name__ == "__main__":
    main()
