"""Merge record_auto datasets (same rig, same target style) into one, keeping their sidecar files.

    python scripts/meca500/merge_autocollect.py --dst NAME SRC [SRC ...]

The episodes of each source follow the previous ones, in order. autocollect_episodes.jsonl and
autocollect/excluded_episodes.json are carried over with episode indices shifted to match, and
each episode's record gets "source": the dataset it came from. Raw frames are not copied.
Merge the recorded (absolute) datasets, then convert the result (convert_to_delta_actions.py,
convert_to_tip_actions.py), so every episode goes through the same conversion.
"""

import argparse
import json
import shutil
from pathlib import Path

from lerobot.datasets.dataset_tools import merge_datasets
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.constants import HF_LEROBOT_HOME

USER = "AdamAxelrod"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dst", required=True)
    ap.add_argument("--force", action="store_true", help="overwrite the destination if it exists")
    ap.add_argument("srcs", nargs="+")
    args = ap.parse_args()

    home = Path(HF_LEROBOT_HOME) / USER
    dst = home / args.dst
    if dst.exists():
        if not args.force:
            raise SystemExit(f"{dst} exists; pass --force to overwrite it.")
        shutil.rmtree(dst)
    datasets = [LeRobotDataset(f"{USER}/{s}", root=home / s) for s in args.srcs]
    merge_datasets(datasets, output_repo_id=f"{USER}/{args.dst}", output_dir=dst)

    records, excluded, offset = [], [], 0
    for s, ds in zip(args.srcs, datasets, strict=True):
        root = home / s
        for line in (root / "autocollect_episodes.jsonl").read_text().splitlines():
            if line.strip():
                r = json.loads(line)
                records.append({**r, "episode_index": r["episode_index"] + offset, "source": s})
        excl = root / "autocollect" / "excluded_episodes.json"
        if excl.exists():
            excluded += [e + offset for e in json.loads(excl.read_text())["excluded"]]
        offset += ds.meta.total_episodes
    (dst / "autocollect_episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records), newline="\n")
    (dst / "autocollect").mkdir(exist_ok=True)
    (dst / "autocollect" / "excluded_episodes.json").write_text(
        json.dumps({"excluded": excluded, "reason": f"merged from {args.srcs}; see each source"}, indent=2),
        newline="\n",
    )
    merged = LeRobotDataset(f"{USER}/{args.dst}", root=dst)
    print(f"{dst}: {merged.meta.total_episodes} episodes, {merged.meta.total_frames} frames, excluded {excluded}")
    if len(records) != merged.meta.total_episodes:
        raise SystemExit(f"{len(records)} episode records for {merged.meta.total_episodes} episodes")


if __name__ == "__main__":
    main()
