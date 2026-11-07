"""Control-loop timing of eval_auto policy episodes, from each epNNN/traj.npz's step times.

    python scripts/meca500/loop_timing.py outputs/eval_auto/<run> [<run> ...]

Per episode: the median loop rate, and the stalls - steps that took more than STALL_FACTOR control
periods (e.g. waiting for asr_bridge.GPU_LOCK while the tip watcher runs the detector) - as a
count, their total extra time beyond one period, and the longest. Plus time to view for *_asr runs.
"""

import json
import sys
from pathlib import Path

import numpy as np

FPS = 20
STALL_FACTOR = 1.5


def main() -> None:
    for run in map(Path, sys.argv[1:]):
        rows = {}
        if (run / "results.jsonl").exists():
            rows = {json.loads(line)["episode"]: json.loads(line) for line in (run / "results.jsonl").read_text().splitlines() if line.strip()}
        print(f"== {run.name}")
        for ep in sorted(run.glob("ep*")):
            f = ep / "traj.npz"
            if not f.exists():
                continue
            with np.load(f) as z:
                if "t" not in z or len(z["t"]) < 3:
                    print(f"  {ep.name}: no policy steps")
                    continue
                dt = np.diff(z["t"])
            period = 1 / FPS
            stall = dt > STALL_FACTOR * period
            r = rows.get(int(ep.name[2:]), {})
            print(
                f"  {ep.name}: {len(dt) + 1} steps, {1 / np.median(dt):.1f} Hz median | stalls {int(stall.sum())}, "
                f"extra {float(np.sum(dt[stall] - period)):.2f} s, longest {float(dt.max()):.2f} s | "
                f"view {r.get('time_to_view_s')} s | {r.get('reason', '')[:40]}"
            )


if __name__ == "__main__":
    main()
