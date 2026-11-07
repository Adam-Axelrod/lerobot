"""One policy_asr (or scripted_asr) trial as a paper figure: four microscope frames and the tip speed.

    python scripts/meca500/trial_figure.py outputs/eval_auto/<run> --episode 3 [--out fig.pdf]

Top: four frames from the trial's episode.mp4, with the target ring drawn on -
    (a) the tip entering the view (the detector's first sighting while the policy drives),
    (b) the handover,
    (c) mid-servo,
    (d) the final touch (epNNN/final_overlay.png, full resolution).
Bottom: tip speed (mm/s, forward kinematics on the logged joints, 0.25 s moving average) against
time, with the handover and the stage boundaries marked. 3.5 in wide (one IEEE column) PDF; the
frames are downsampled so the file stays well under 300 KB.
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from lerobot.robots.meca500.autocollect.bench import BenchConfig

RIG = "meca500_basler"
WIDTH_IN = 3.5
FRAME_W_PX = 160  # each frame's width in the PDF: ~190 dpi at 0.85 in, keeps the PDF under 250 KB
SMOOTH_S = 0.25


def frame_at(video: Path, times: list[float], t: float) -> np.ndarray:
    """The video frame nearest time t (RGB)."""
    i = int(np.argmin(np.abs(np.asarray(times) - t)))
    cap = cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_POS_FRAMES, i)
    ok, f = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"could not read frame {i} of {video}")
    return cv2.cvtColor(f, cv2.COLOR_BGR2RGB)


def small(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    return cv2.resize(img, (FRAME_W_PX, round(h * FRAME_W_PX / w)), interpolation=cv2.INTER_AREA)


def moments(row: dict) -> dict[str, float]:
    """Times (s since the drive started) of the four panels."""
    st = {s["stage"]: s for s in row["stages"]}
    approach = st.get("approach", {})
    seen = [d["t"] for d in approach.get("detections", []) if d.get("px") is not None]
    t_hand = row.get("time_to_view_s", st["handover"]["t_s"])
    servo = st.get("servo", {"t_s": t_hand, "duration_s": 0.0})
    return {
        "tip enters view": seen[0] if seen else t_hand,
        "handover": t_hand,
        "servo": servo["t_s"] + servo["duration_s"] / 2,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = [json.loads(line) for line in (args.run / "results.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    row = next(r for r in rows if r["episode"] == args.episode)
    if "stages" not in row or not any(s["stage"] == "handover" for s in row["stages"]):
        raise SystemExit(f"episode {args.episode} has no handover: pick a policy_asr / scripted_asr trial that reached view")
    ep = args.run / f"ep{args.episode:03d}"
    times = json.loads((ep / "episode_t.json").read_text())
    t = moments(row)
    final = cv2.cvtColor(cv2.imread(str(ep / "final_overlay.png")), cv2.COLOR_BGR2RGB)
    panels = [(k, small(frame_at(ep / "episode.mp4", times, v))) for k, v in t.items()]
    panels.append(("touch", small(final)))

    kin = BenchConfig.load(RIG).kinematics()
    with np.load(ep / "traj.npz") as z:
        ts, qs = z["obs_t"], z["obs_q"]
    tip = np.array([kin.tip(q)[:3, 3] for q in qs])
    speed = np.r_[0.0, np.linalg.norm(np.diff(tip, axis=0), axis=1) / np.maximum(np.diff(ts), 1e-3)]
    k = max(1, int(round(SMOOTH_S / max(np.median(np.diff(ts)), 1e-3))))
    speed = np.convolve(speed, np.ones(k) / k, mode="same")

    plt.rcParams.update({"font.size": 7, "pdf.fonttype": 42, "axes.linewidth": 0.5})
    h_img = panels[0][1].shape[0] / panels[0][1].shape[1] * WIDTH_IN / 4
    fig = plt.figure(figsize=(WIDTH_IN, h_img + 1.25))
    gs = fig.add_gridspec(2, 4, height_ratios=[h_img, 1.0], hspace=0.35, wspace=0.04)
    for i, (label, img) in enumerate(panels):
        ax = fig.add_subplot(gs[0, i])
        ax.imshow(img, interpolation="none")
        ax.set_axis_off()
        ax.set_title(f"({'abcd'[i]}) {label}", fontsize=6, pad=2)
    ax = fig.add_subplot(gs[1, :])
    ax.plot(ts, speed, lw=0.7, color="0.2")
    for s in row["stages"][1:]:
        ax.axvline(s["t_s"], lw=0.4, ls=":", color="0.6")
    ax.axvline(t["handover"], lw=0.8, color="tab:red")
    ax.text(t["handover"], ax.get_ylim()[1] * 0.92, " handover", color="tab:red", fontsize=6, va="top")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("tip speed (mm/s)")
    ax.set_xlim(0, ts[-1] if len(ts) else 1)
    ax.spines[["top", "right"]].set_visible(False)
    out = args.out or ep / "trial_figure.pdf"
    fig.savefig(out, bbox_inches="tight", pad_inches=0.01, dpi=300)
    print(f"{out} ({out.stat().st_size / 1024:.0f} KB)")


if __name__ == "__main__":
    main()
