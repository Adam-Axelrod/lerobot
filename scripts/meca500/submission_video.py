"""The ISMR submission video: whole eval_auto trials, captioned by stage, sped up where slow.

    python scripts/meca500/submission_video.py --out outputs/figures/ismr_video.mp4 \
        --clip <run>:<episode>:<title>[#<caption>] [--clip ...]

Each clip is a trial's epNNN/episode.mp4 (the microscope view with the target ring), its stages
from results.jsonl as captions, and its final frame (epNNN/final_overlay.png) held with the
final distance. Stages are played at STAGE_SPEED (approach and search fast, the finish at real
speed); a trial without stages (scripted, expert) plays at PLAIN_SPEED. H.264 via PyAV, 720x480,
30 fps: small enough for the 10 MB limit at a few minutes.
"""

import argparse
import json
from pathlib import Path

import av
import cv2
import numpy as np

W, H, FPS = 720, 480, 30
STAGE_SPEED = {"approach": 2.0, "search": 2.0, "handover": 1.0, "lower": 1.0, "servo": 1.0, "touch": 1.0}
PLAIN_SPEED = 2.0
CAPTION = {
    "approach": "learned approach (ACT)",
    "search": "tip out of view: search",
    "handover": "tip in view: handover to visual servo",
    "lower": "lower to 150 um above the dish",
    "servo": "visual servo onto the target",
    "touch": "touch",
}
TITLE_S, HOLD_S = 3.0, 2.5


def fit(img: np.ndarray) -> np.ndarray:
    """Letterboxed into W x H (RGB)."""
    h, w = img.shape[:2]
    s = min(W / w, H / h)
    small = cv2.resize(img, (round(w * s), round(h * s)), interpolation=cv2.INTER_AREA)
    out = np.zeros((H, W, 3), np.uint8)
    y, x = (H - small.shape[0]) // 2, (W - small.shape[1]) // 2
    out[y : y + small.shape[0], x : x + small.shape[1]] = small
    return out


def text(img: np.ndarray, lines: list[str], y0: int = 30, scale: float = 0.8) -> np.ndarray:
    img = img.copy()
    for i, line in enumerate(lines):
        y = y0 + i * int(36 * scale)
        cv2.putText(img, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def card(lines: list[str]) -> np.ndarray:
    img = np.zeros((H, W, 3), np.uint8)
    for i, line in enumerate(lines):
        (tw, _), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.putText(img, line, ((W - tw) // 2, H // 2 - 20 * len(lines) + 44 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def read_frames(path: Path) -> list[np.ndarray]:
    cap, frames = cv2.VideoCapture(str(path)), []
    while True:
        ok, f = cap.read()
        if not ok:
            return frames
        frames.append(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))


def clip_frames(run: Path, episode: int, title: str, n0: int = 0) -> list[np.ndarray]:
    row = next(
        r for r in map(json.loads, (run / "results.jsonl").read_text(encoding="utf-8").splitlines()) if r["episode"] == episode
    )
    ep = run / f"ep{episode:03d}"
    frames, times = read_frames(ep / "episode.mp4"), json.loads((ep / "episode_t.json").read_text(encoding="utf-8"))
    # eval_auto logs "handover" after a search with the search's own t_s (its clock starts before
    # the search), so it would caption the whole spiral: a stage that starts with an earlier one
    # begins at that one's end instead.
    stages = []
    for s in row.get("stages", []):
        t0, t1 = s["t_s"], s["t_s"] + s["duration_s"]
        for p in stages:
            if abs(p["t_s0"] - s["t_s"]) < 1e-6:
                t0 = max(t0, p["t_s1"])
        if t1 > t0:
            stages.append({"stage": s["stage"], "t_s": t0, "t_s1": t1, "t_s0": s["t_s"]})
    for a, b in zip(stages, stages[1:]):
        if b["t_s"] < a["t_s1"] - 0.02:  # logged to 0.01 s
            print(f"  WARNING ep {episode}: {b['stage']} starts {a['t_s1'] - b['t_s']:.2f} s before {a['stage']} ends")

    def stage_at(t: float):
        cur = None
        for s in stages:
            if t >= s["t_s"]:
                cur = s["stage"]
        return cur

    title, _, caption = title.partition("#")
    out = [card(title.split("|"))] * int(TITLE_S * FPS)
    # Caption timeline in output-video seconds: [caption, speed, start, end].
    timeline = [["title card", None, n0 / FPS, (n0 + len(out)) / FPS]]
    # Resample on an output clock that runs at each stage's speed.
    t_video, i, end = times[0], 0, times[min(len(times), len(frames)) - 1]
    while t_video <= end:
        while i + 1 < len(frames) and i + 1 < len(times) and times[i + 1] <= t_video:
            i += 1
        st = stage_at(t_video)
        speed = STAGE_SPEED.get(st, PLAIN_SPEED) if stages else PLAIN_SPEED
        lines = [CAPTION[st] if st else ""] if stages else [caption or title.split("|")[0]]
        if timeline[-1][:2] != [lines[0], speed]:
            timeline.append([lines[0], speed, (n0 + len(out)) / FPS, None])
        lines.append(f"t = {t_video:4.1f} s   ({speed:g}x)")
        out.append(text(fit(frames[i]), lines))
        timeline[-1][3] = (n0 + len(out)) / FPS
        t_video += speed / FPS
    final = cv2.cvtColor(cv2.imread(str(ep / "final_overlay.png")), cv2.COLOR_BGR2RGB)
    d = row.get("vision_error_um")
    d = row.get("fk_error_um") if d is None or row.get("fk_error_um") is None or abs(d - row["fk_error_um"]) > 500 else d
    timeline.append(["final", None, (n0 + len(out)) / FPS, None])
    out += [text(fit(final), ["final", f"tip to target {d:.0f} um" if d is not None else ""])] * int(HOLD_S * FPS)
    timeline[-1][3] = (n0 + len(out)) / FPS
    print(f"{run.name} ep {episode}:")
    for cap, speed, a, b in timeline:
        print(f"  {a:6.2f}-{b:6.2f} s  {'' if speed is None else f'{speed:g}x':4s}  {cap or '(no stage)'}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--clip", action="append", required=True, help="run:episode:title (| splits title lines; #caption sets a plain clip's in-clip caption)")
    ap.add_argument("--intro", default="", help="opening card lines, | separated")
    args = ap.parse_args()
    frames = [card(args.intro.split("|"))] * int(TITLE_S * FPS) if args.intro else []
    for c in args.clip:
        run, episode, title = c.split(":", 2) if c.count(":") >= 2 else (*c.split(":", 1), "")  # relative run paths
        frames += clip_frames(Path(run), int(episode), title, len(frames))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with av.open(str(args.out), "w") as container:
        stream = container.add_stream("libx264", rate=FPS)
        stream.width, stream.height, stream.pix_fmt = W, H, "yuv420p"
        stream.options = {"crf": "26", "preset": "slow"}
        for f in frames:
            for packet in stream.encode(av.VideoFrame.from_ndarray(f, format="rgb24")):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    mb = args.out.stat().st_size / 1e6
    print(f"{args.out}: {len(frames) / FPS:.1f} s, {mb:.2f} MB, {W}x{H} @ {FPS} fps")


if __name__ == "__main__":
    main()
