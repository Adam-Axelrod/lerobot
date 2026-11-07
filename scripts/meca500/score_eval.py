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
"close" is a miss at SUCCESS_UM judged within CLOSE_UM (the second column). Without a verdict the
second column uses the distance; a "miss" at or under SUCCESS_UM (e.g. never touched) is never close.

--dataset scores the automatic-collection row from what the recorder logged, against all saved
episodes: autocollect_episodes.jsonl for the final error, and the commanded joints in the dataset
for the motion. Episodes left out of training still count here. --from-episode reports only the
batches after a fix, which the paper must then say. --verdicts takes a JSON file of by-eye verdicts
for it, in the form above, since the detector can be misled by debris.

policy_asr runs (eval_auto) get a second row, "<session> without search": the trials that needed
the search counted as failures with no distance, since without it they end with the tip out of
view. Under the rows: how many searched, and the stage each failure stopped at.

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
# Second, looser column ("close enough", Adam 2026-11-06): the pipette ends on the target.
# SUCCESS_UM stays the primary measure.
CLOSE_UM = 100.0
FPS = 20


def reached_view(row: dict, field) -> bool | None:
    """Did the tip get under the microscope? policy_asr / scripted_asr: the handover happened
    (searched or not). Other methods: the final tip, by forward kinematics, is in the lit field."""
    if "stages" in row:
        return any(s["stage"] == "handover" for s in row["stages"])
    if field is None or row.get("fk_tip_px") is None:
        return None
    centre, radius = field
    return bool(np.linalg.norm(np.subtract(row["fk_tip_px"], centre)) <= radius)


def slide_of(row: dict) -> float:
    """The measured physical slide of the robot (mm) the trial ran at; 0 at the original position."""
    return float((row.get("slide") or {}).get("slide_norm_mm", 0.0))


def score_trial(row: dict, verdict, kin, field=None) -> dict:
    """One results.jsonl row, plus its by-eye verdict and its path metrics."""
    distance = row.get("vision_error_um")
    if isinstance(verdict, int | float) and not isinstance(verdict, bool):
        distance = float(verdict)
    reason = str(row.get("reason", ""))
    stopped = reason.startswith("stopped")
    press = stopped and "below the dish" in reason
    collision = stopped and not press
    # *_asr: the approach's own collision-check stop counts too, whatever came after (a search).
    approach = str(((row.get("stages") or [{}])[0]).get("reason", ""))
    if approach.startswith("stopped") and "below the dish" not in approach:
        collision = True
    # Of those, a classical move (search / lower / servo / touch) the collision check refused before
    # it started - counted in Coll. % like a policy stopped by the check, and footnoted.
    refusal = collision and "would collide" in reason
    if verdict == "hit":
        success = True
    elif verdict == "miss":
        success = False
    else:
        success = distance is not None and distance <= SUCCESS_UM and not collision
    if verdict in ("hit", "close"):
        close = True
    elif verdict == "miss" and distance is not None and distance <= SUCCESS_UM:
        close = False  # missed for another reason than distance (e.g. never touched)
    else:
        close = distance is not None and distance <= CLOSE_UM and not collision
    out = {
        "episode": row["episode"],
        "success": success,
        "close": close,
        "search_used": bool(row.get("search_used", False)),
        "view": reached_view(row, field),
        "slide_mm": slide_of(row),
        "failure_stage": row.get("failure_stage"),
        "distance_um": distance,
        "collision": collision,
        "refusal": refusal,
        "press": press,
    }
    path = row.get("_path")
    if path is not None and len(path[0]) >= 4:
        t, q = path
        out |= path_metrics(t, np.array([kin.tip(v)[:3, 3] for v in q]), fs=FPS, arrive_um=SUCCESS_UM)
    return out


def without_search(trial: dict) -> dict:
    """A policy_asr trial scored as if the search did not exist: one that needed it fails."""
    if not trial["search_used"]:
        return trial
    return trial | {"success": False, "close": False, "distance_um": None, "failure_stage": "view", "view": False}


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
    views = [t["view"] for t in trials if t.get("view") is not None]
    return {
        "n": len(trials),
        "view": sum(views) if views else None,
        "view_n": len(views),
        "successes": len(hits),
        "close": sum(t["close"] for t in trials),
        "collisions": sum(t["collision"] for t in trials),
        "refusals": sum(t.get("refusal", False) for t in trials),
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
        f"<= {CLOSE_UM:.0f} um {s['close']:>3}/{s['n']:<3} {100 * s['close'] / max(s['n'], 1):>4.0f}%  "
        f"{s['distance_median']:>6.0f} [{s['distance_q1']:.0f}, {s['distance_q3']:.0f}] um  "
        f"{s['time_median']:>6.1f} s  SPARC {s['sparc_median']:>6.2f}  economy {100 * s['economy_median']:>4.0f}%  "
        f"collisions {s['collisions']} (refused classical moves {s['refusals']})  press stops {s['press_stops']}  no distance {s['no_distance']}"
    )


# The paper's failure stages, in order. "view" (the tip never seen, no search) is the approach's.
STAGES = ("approach", "search", "lower", "servo", "touch")


def stage_counts(trials: list[dict]) -> dict:
    out = dict.fromkeys(STAGES, 0)
    for t in trials:
        stage = t.get("failure_stage")
        if stage:
            key = "approach" if stage == "view" else stage
            out[key] = out.get(key, 0) + 1
    return out


def pct(k, n) -> str:
    return "--" if not n or k is None else f"{100 * k / n:.0f}"


def num(v, fmt=".0f") -> str:
    return "--" if v is None or not np.isfinite(v) else format(v, fmt)


def latex_main_row(name: str, s: dict) -> str:
    """Main table: View %, Coll. %, Within 50 um %, Within 100 um %, median [IQR] um, time s, SPARC."""
    return (
        f"{name} & {pct(s['view'], s['view_n'])} & {pct(s['collisions'], s['n'])} & {pct(s['successes'], s['n'])} & "
        f"{pct(s['close'], s['n'])} & {num(s['distance_median'])} [{num(s['distance_q1'])}, {num(s['distance_q3'])}] & "
        f"{num(s['time_median'], '.1f')} & {num(s['sparc_median'], '.2f')} \\\\  % n={s['n']}, collision-check refusals of classical moves {s['refusals']}"
    )


def latex_moved_row(name: str, by_slide: dict[float, dict]) -> str:
    """Moved-robot table: success % and median distance at each slide, in order of distance."""
    cells = [f"{pct(s['successes'], s['n'])} & {num(s['distance_median'])}" for _, s in sorted(by_slide.items())]
    slides = ", ".join(f"{d:.1f} mm n={s['n']}" for d, s in sorted(by_slide.items()))
    return f"{name} & " + " & ".join(cells) + f" \\\\  % {slides}"


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


def dataset_trials(root: Path, from_episode: int, kin, verdicts: dict | None = None) -> list[dict]:
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
        trials.append(score_trial(row, (verdicts or {}).get(str(ep)), kin))
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
    parser.add_argument(
        "--verdicts", type=Path, help="--dataset: a JSON file of by-eye verdicts, keyed by episode"
    )
    parser.add_argument("--allow-different-targets", action="store_true")
    args = parser.parse_args()
    if not args.runs and args.dataset is None:
        parser.error("give at least one run folder, or --dataset")
    kin = BenchConfig.load(RIG).kinematics()

    target = BenchConfig.load(RIG).section("target")
    field = (target["field_centre_px"], target["field_radius_px"]) if "field_centre_px" in target else None
    sessions: dict[str, list[dict]] = {}
    for run in args.runs:
        for row in load_run(run):
            # A physical slide keeps the session name: its trials are a row of their own.
            slid = f" @ {slide_of(row):.1f} mm" if slide_of(row) else ""
            sessions.setdefault(session_of(run) + slid, []).append(row)
    problems = check_same_targets(sessions)
    if problems and not args.allow_different_targets:
        sys.exit("These runs did not face the same test set:\n  " + "\n  ".join(problems[:10]))
    latex, moved = [], {}
    for name, rows in sessions.items():
        trials = [score_trial(r, r["_verdict"], kin, field) for r in rows]
        s = summarise(trials)
        print(format_row(name, s))
        latex.append(latex_main_row(name, s))
        base = name.split(" @ ")[0]
        moved.setdefault(base, {})[trials[0]["slide_mm"] if trials else 0.0] = s
        if any("search_used" in r for r in rows):
            # policy_asr: the same trials as if there were no search - one that needed it ends,
            # tip out of view, where the policy left it.
            ws = [without_search(t) for t in trials]
            print(format_row(f"{name} without search", summarise(ws)))
            latex.append(latex_main_row(f"{name} without search", summarise(ws)))
            moved.setdefault(f"{base} without search", {})[trials[0]["slide_mm"] if trials else 0.0] = summarise(ws)
            print(
                f"    searched {sum(t['search_used'] for t in trials)}/{len(trials)}; failed at "
                f"{stage_counts(trials)}; without search {stage_counts(ws)}"
            )
        unchecked = [r["episode"] for r in rows if r["_verdict"] is None]
        if unchecked:
            print(f"    not checked by eye yet (verdicts.json): {unchecked}")
    if latex:
        print("\n% main table: View % & Coll. % & <=50 um % & <=100 um % & median [IQR] um & time s & SPARC")
        print("\n".join(latex))
        print("% moved table: success % & median um at each slide (0, 2, 4 mm)")
        print("\n".join(latex_moved_row(k, v) for k, v in moved.items()))
    if args.dataset is not None:
        label = f"automatic collection (dataset, ep >= {args.from_episode})"
        verdicts = json.loads(args.verdicts.read_text(encoding="utf-8")) if args.verdicts else None
        print(format_row(label, summarise(dataset_trials(args.dataset, args.from_episode, kin, verdicts))))


if __name__ == "__main__":
    main()
