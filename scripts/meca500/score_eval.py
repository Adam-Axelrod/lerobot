"""Turn eval_auto.py results into rows of the paper's tables. Needs no robot.

    python scripts/meca500/score_eval.py outputs/eval_auto/<run> [<run> ...]
    python scripts/meca500/score_eval.py --dataset <dataset root> [--from-episode 50]

One row per run folder. Runs sharing a session name (a session continued with FIRST_EPISODE) are
joined into one row. Runs that were meant to share a test set must have the same target at every
seed: if two runs disagree (a recalibration can change what plans), this refuses rather than
compare them on different tests. Use --allow-different-targets to look at them anyway.

By-eye verdicts. The detector can misplace a tip resting on the dish, so eval_auto says to check
each epNNN/final_overlay.png. Put the verdicts in <run>/verdicts.json, keyed by episode:
    {"3": "hit", "7": "miss", "8": 62.0}
"hit"/"miss" override the success decision; a number replaces the measured distance in um.

--dataset scores the automatic-collection row from what the recorder logged, against all saved
episodes: autocollect_episodes.jsonl for the final error, and the commanded joints in the dataset
for the motion. Episodes left out of training still count here. --from-episode reports only the
batches after a fix, which the paper must then say.

Definitions:
    success  final distance <= SUCCESS_UM, no collision stop (or the by-eye verdict)
    collision  a stop on the collision model. Reaching the press limit under the dish is not one:
             it is how most policy touches end, so it is counted separately and the trial is
             judged on where the tip landed
    distance median [IQR] of the final distance, by the tip detector on the frame the method saw
    time     median arrival_s of the successful trials (metrics.path_metrics)
    SPARC, economy   median over all trials with a path
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.metrics import path_metrics

RIG = "meca500_basler"
SUCCESS_UM = 50.0
FPS = 20


def score_trial(row: dict, verdict, kin) -> dict:
    """One results.jsonl row, plus its by-eye verdict and its path metrics."""
    distance = row.get("vision_error_um")
    if isinstance(verdict, int | float) and not isinstance(verdict, bool):
        distance = float(verdict)
    reason = str(row.get("reason", ""))
    stopped = reason.startswith("stopped")
    press = stopped and "below the dish" in reason
    collision = stopped and not press
    if verdict == "hit":
        success = True
    elif verdict == "miss":
        success = False
    else:
        success = distance is not None and distance <= SUCCESS_UM and not collision
    out = {
        "episode": row["episode"],
        "success": success,
        "distance_um": distance,
        "collision": collision,
        "press": press,
    }
    path = row.get("_path")
    if path is not None and len(path[0]) >= 4:
        t, q = path
        out |= path_metrics(t, np.array([kin.tip(v)[:3, 3] for v in q]), fs=FPS, arrive_um=SUCCESS_UM)
    return out


def load_run(run: Path) -> list[dict]:
    rows = [
        json.loads(line)
        for line in (run / "results.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    for row in rows:
        traj = run / f"ep{row['episode']:03d}" / "traj.npz"
        if traj.exists():
            with np.load(traj) as z:
                # Runs from before ObservationTap logged only the policy's own t and q_meas.
                row["_path"] = (z["obs_t"], z["obs_q"]) if "obs_t" in z else (z["t"], z["q_meas"])
    verdict_file = run / "verdicts.json"
    verdicts = json.loads(verdict_file.read_text(encoding="utf-8")) if verdict_file.exists() else {}
    for row in rows:
        row["_verdict"] = verdicts.get(str(row["episode"]))
    return rows


def session_of(run: Path) -> str:
    return re.sub(r"_\d{8}-\d{6}$", "", run.name)


def summarise(trials: list[dict]) -> dict:
    def med(key, subset=trials):
        vals = np.array([t[key] for t in subset if t.get(key) is not None], float)
        vals = vals[np.isfinite(vals)]
        return float(np.median(vals)) if len(vals) else float("nan")

    dist = np.array([t["distance_um"] for t in trials if t["distance_um"] is not None], float)
    hits = [t for t in trials if t["success"]]
    return {
        "n": len(trials),
        "successes": len(hits),
        "collisions": sum(t["collision"] for t in trials),
        "press_stops": sum(t["press"] for t in trials),
        "no_distance": sum(t["distance_um"] is None for t in trials),
        "distance_median": float(np.median(dist)) if len(dist) else float("nan"),
        "distance_q1": float(np.percentile(dist, 25)) if len(dist) else float("nan"),
        "distance_q3": float(np.percentile(dist, 75)) if len(dist) else float("nan"),
        "time_median": med("arrival_s", hits),
        "sparc_median": med("sparc"),
        "economy_median": med("economy"),
    }


def format_row(name: str, s: dict) -> str:
    return (
        f"{name:<44} {s['successes']:>3}/{s['n']:<3} {100 * s['successes'] / max(s['n'], 1):>4.0f}%  "
        f"{s['distance_median']:>6.0f} [{s['distance_q1']:.0f}, {s['distance_q3']:.0f}] um  "
        f"{s['time_median']:>6.1f} s  SPARC {s['sparc_median']:>6.2f}  economy {100 * s['economy_median']:>4.0f}%  "
        f"collisions {s['collisions']}  press stops {s['press_stops']}  no distance {s['no_distance']}"
    )


def check_same_targets(sessions: dict[str, list[dict]]) -> list[str]:
    """Seeds whose target differs between sessions."""
    by_seed: dict[int, tuple[str, list[float]]] = {}
    problems = []
    for name, rows in sessions.items():
        for row in rows:
            first = by_seed.setdefault(row["seed"], (name, row["target_px"]))
            if np.abs(np.subtract(first[1], row["target_px"])).max() > 0.01:
                problems.append(f"seed {row['seed']}: {first[1]} in {first[0]}, {row['target_px']} in {name}")
    return problems


def dataset_trials(root: Path, from_episode: int, kin) -> list[dict]:
    """Automatic collection from its own logs: logged final error, commanded joints as the path."""
    import pandas as pd

    meta = root / "autocollect_episodes.jsonl"
    if not meta.exists():
        meta = root / "autocollect" / "autocollect_episodes.jsonl"
    rows = [json.loads(line) for line in meta.read_text(encoding="utf-8").splitlines() if line.strip()]
    frames = pd.concat(
        pd.read_parquet(f, columns=["episode_index", "action"])
        for f in sorted((root / "data").rglob("*.parquet"))
    )
    trials = []
    for meta_row in rows:
        ep = meta_row["episode_index"]
        if ep < from_episode:
            continue
        actions = np.stack(frames.loc[frames["episode_index"] == ep, "action"].to_numpy())[:, :6]
        row = {
            "episode": ep,
            "reason": "finished",
            "vision_error_um": meta_row.get("final_error_um"),
            "_path": (np.arange(len(actions)) / FPS, actions),
        }
        trials.append(score_trial(row, None, kin))
    return trials


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("runs", nargs="*", type=Path, help="outputs/eval_auto/<run> folders")
    parser.add_argument(
        "--dataset", type=Path, help="a record_auto dataset root, for the automatic-collection row"
    )
    parser.add_argument(
        "--from-episode", type=int, default=0, help="--dataset: only episodes from this index"
    )
    parser.add_argument("--allow-different-targets", action="store_true")
    args = parser.parse_args()
    if not args.runs and args.dataset is None:
        parser.error("give at least one run folder, or --dataset")
    kin = BenchConfig.load(RIG).kinematics()

    sessions: dict[str, list[dict]] = {}
    for run in args.runs:
        sessions.setdefault(session_of(run), []).extend(load_run(run))
    problems = check_same_targets(sessions)
    if problems and not args.allow_different_targets:
        sys.exit("These runs did not face the same test set:\n  " + "\n  ".join(problems[:10]))
    for name, rows in sessions.items():
        print(format_row(name, summarise([score_trial(r, r["_verdict"], kin) for r in rows])))
        unchecked = [r["episode"] for r in rows if r["_verdict"] is None]
        if unchecked:
            print(f"    not checked by eye yet (verdicts.json): {unchecked}")
    if args.dataset is not None:
        label = f"automatic collection (dataset, ep >= {args.from_episode})"
        print(format_row(label, summarise(dataset_trials(args.dataset, args.from_episode, kin))))


if __name__ == "__main__":
    main()
