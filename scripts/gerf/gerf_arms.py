"""Shared settings for the GERF SO-101 arms, so every machine uses the same calibration.

Calibration lives in the repo (scripts/gerf/calibration/) instead of the per-machine
~/.cache/huggingface/lerobot/calibration, so pulling on another computer works
without recalibrating. Recalibrating from any machine writes back here; commit it.

Serial ports differ per machine. Find yours with `lerobot-find-port` and fill in
the entry for your platform below. They follow the USB board, not the arm.
"""

import sys
from pathlib import Path

FOLLOWER_ID = "my_yellow_robot_arm"
LEADER_ID = "my_purple_leader_arm"

CALIBRATION_DIR = Path(__file__).resolve().parent / "calibration"
FOLLOWER_CALIBRATION_DIR = CALIBRATION_DIR / "robots" / "so_follower"
LEADER_CALIBRATION_DIR = CALIBRATION_DIR / "teleoperators" / "so_leader"

PORTS = {
    "win32": {"follower": "COM4", "leader": "COM10"},
    # macOS ports look like /dev/tty.usbmodemXXXX; fill in from `lerobot-find-port`.
    "darwin": {"follower": None, "leader": None},
}


def port(arm: str) -> str:
    value = PORTS.get(sys.platform, {}).get(arm)
    if value is None:
        raise RuntimeError(
            f"No {arm} port set for platform '{sys.platform}'. "
            f"Run `lerobot-find-port` and add it to PORTS in {Path(__file__).name}."
        )
    return value
