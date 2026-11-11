"""Copy a recorded dataset with its camera cropped around each episode's target.

    python scripts/meca500/crop_dataset.py --src NAME [--size 256] [--force]

Writes AdamAxelrod/<NAME with _crop<size> before its _js_tip_delta / _tip_delta / _delta suffix>,
so eval_auto.py still sees an action-space suffix it knows and reads the crop size from the name.
Each frame becomes a size x size crop centred on the episode's target_px
(autocollect_episodes.jsonl), cut by autocollect/overlay.crop_at - the function eval_auto's
TargetCropStep uses live. Actions, state and episode boundaries are copied unchanged, and so is
autocollect_episodes.jsonl. The ring is already drawn into the recorded frames, so it sits at the
centre of every crop; the tip's place in the crop is the tip-to-target offset.

Why: offline (THEORIES.md theory 1) a crop policy followed a moved ring further than a full-frame
one, and the 256 px default keeps a tip 1.5 mm from the target (113 px) inside the crop.
"""

import argparse
import json
import re
import shutil

import av
import numpy as np
import pandas as pd

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.meca500.autocollect.overlay import crop_at
from lerobot.utils.constants import HF_LEROBOT_HOME

USER = "AdamAxelrod"
DEFAULT_KEYS = ("timestamp", "frame_index", "episode_index", "index", "task_index")


def crop_name(src: str, size: int) -> str:
    m = re.search(r"(_js_tip_delta|_tip_delta|_delta)$", src)
    if not m:
        raise SystemExit(f"{src} has no _delta suffix: crop a delta dataset (eval_auto.py sends deltas).")
    return f"{src[: m.start()]}_crop{size}{m.group(1)}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", required=True)
    ap.add_argument("--size", type=int, default=256)
    ap.add_argument("--force", action="store_true", help="replace an existing output")
    args = ap.parse_args()

    src_root = HF_LEROBOT_HOME / USER / args.src
    name = crop_name(args.src, args.size)
    root = HF_LEROBOT_HOME / USER / name
    if root.exists():
        if not args.force:
            raise SystemExit(f"{root} exists; --force replaces it.")
        shutil.rmtree(root)
    info = json.loads((src_root / "meta/info.json").read_text())
    fps = info["fps"]
    cams = [k for k, f in info["features"].items() if f["dtype"] == "video"]
    if len(cams) != 1:
        raise SystemExit(f"expected one camera, found {cams}")
    cam = cams[0]
    features = {k: {kk: vv for kk, vv in f.items() if kk != "info"} for k, f in info["features"].items() if k not in DEFAULT_KEYS}
    features[cam] = {"dtype": "video", "shape": [args.size, args.size, 3], "names": ["height", "width", "channels"]}

    data = pd.concat(pd.read_parquet(p) for p in sorted((src_root / "data").glob("*/*.parquet")))
    data = data.sort_values("index").reset_index(drop=True)
    meta = pd.concat(pd.read_parquet(p) for p in sorted((src_root / "meta/episodes").glob("*/*.parquet")))
    meta = meta.sort_values("episode_index").reset_index(drop=True)
    tasks = pd.read_parquet(src_root / "meta/tasks.parquet")
    task_of = {int(i): str(t) for t, i in zip(tasks.index, tasks.task_index)}
    side = {
        (r := json.loads(line))["episode_index"]: r
        for line in (src_root / "autocollect_episodes.jsonl").read_text().splitlines()
        if line.strip()
    }
    rows = {ep: g.reset_index(drop=True) for ep, g in data.groupby("episode_index", sort=True)}

    out = LeRobotDataset.create(f"{USER}/{name}", fps, features=features, root=root, robot_type=info["robot_type"], use_videos=True)
    key = f"videos/{cam}"
    n_done = 0
    for (chunk, file), m in meta.groupby([f"{key}/chunk_index", f"{key}/file_index"], sort=True):
        spans = sorted((float(r[f"{key}/from_timestamp"]), int(r.episode_index), int(r.length)) for _, r in m.iterrows())
        path = src_root / f"videos/{cam}/chunk-{chunk:03d}/file-{file:03d}.mp4"
        frames: dict[int, list] = {}
        with av.open(str(path)) as c:
            for f in c.decode(video=0):
                t = float(f.pts * f.time_base)
                for t0, ep, n in spans:
                    k = int(round((t - t0) * fps))
                    if 0 <= k < n:
                        frames.setdefault(ep, [None] * n)[k] = crop_at(f.to_ndarray(format="rgb24"), side[ep]["target_px"], args.size)
                        if k == n - 1:  # the episode's last frame: write it out
                            write_episode(out, rows[ep], frames.pop(ep), cam, features, task_of)
                            n_done += 1
                        break
        if frames:
            raise SystemExit(f"{path}: episodes {sorted(frames)} did not decode to their last frame")
    out.finalize()
    if n_done != len(meta):
        raise SystemExit(f"wrote {n_done} of {len(meta)} episodes")
    shutil.copy(src_root / "autocollect_episodes.jsonl", root / "autocollect_episodes.jsonl")
    print(f"{root}: {n_done} episodes, {len(data)} frames, {cam} {args.size}x{args.size} around target_px")


def write_episode(out, g: pd.DataFrame, frames: list, cam: str, features: dict, task_of: dict) -> None:
    if any(f is None for f in frames):
        raise SystemExit(f"episode {int(g.episode_index[0])}: {sum(f is None for f in frames)} frames missing")
    for i, img in enumerate(frames):
        frame = {cam: img, "task": task_of[int(g.task_index[i])]}
        for k in features:
            if k != cam:
                frame[k] = np.asarray(g[k][i], dtype=np.float32)
        out.add_frame(frame)
    out.save_episode()


if __name__ == "__main__":
    main()
