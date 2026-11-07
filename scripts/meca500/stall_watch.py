"""Watch a running eval_auto session's heartbeat; exit with an alert if it stalls. Read-only.

    python scripts/meca500/stall_watch.py [<run dir>]     # default: the newest outputs/eval_auto/*

eval_auto's ObservationTap writes <run>/heartbeat.txt about once a second while anything reads
the robot. This exits:
    STALL   no heartbeat for STALL_S (the session is hung: py-spy it, kill it, go_home.py)
    DONE    the eval_auto process has gone (the session ended; read its log)
Run it in the background beside each session, so a stall costs a minute, not forty.
"""

import subprocess
import sys
import time
from pathlib import Path

STALL_S = 60.0
POLL_S = 10.0
START_GRACE_S = 180.0  # model loading and the move home come before the first heartbeat


def eval_running() -> bool:
    out = subprocess.run(
        [
            "powershell", "-NoProfile", "-Command",
            "(Get-CimInstance Win32_Process -Filter \"name='python.exe'\" | "
            "Where-Object { $_.CommandLine -like '*scripts/meca500/eval_auto.py*' -or $_.CommandLine -like '*scripts\\meca500\\eval_auto.py*' }).Count",
        ],
        capture_output=True,
        text=True,
    ).stdout.strip()
    return out not in ("", "0")


def main() -> None:
    if len(sys.argv) > 1:
        run = Path(sys.argv[1])
    else:
        runs = sorted(Path("outputs/eval_auto").glob("*_20??????-??????"), key=lambda p: p.stat().st_mtime)
        run = runs[-1]
    beat = run / "heartbeat.txt"
    t0 = time.time()
    print(f"watching {beat}", flush=True)
    while True:
        time.sleep(POLL_S)
        if not eval_running():
            print(f"DONE: no eval_auto process; last heartbeat {time.time() - beat.stat().st_mtime:.0f} s ago" if beat.exists() else "DONE")
            return
        if not beat.exists():
            if time.time() - t0 > START_GRACE_S:
                print(f"STALL: no heartbeat {START_GRACE_S:.0f} s after start ({run})")
                return
            continue
        age = time.time() - beat.stat().st_mtime
        if age > STALL_S:
            print(f"STALL: heartbeat {age:.0f} s old ({run})")
            return


if __name__ == "__main__":
    main()
