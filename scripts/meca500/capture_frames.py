"""Save microscope frames on a key press, exactly as recording sees them (same camera class,
resolution and stored Basler user set).

Used to collect real-bead images for the digital target: the bead marker is sized and styled
from them, crops of them become the pasted target, and they tune the bead detector that finds
the real target at test time.

    .venv\\Scripts\\python.exe scripts\\meca500\\capture_frames.py

Keys (in the image window): space = save the current frame, q = quit. Frames go to
OUT_DIR as <PREFIX>_<n>.png (RGB, never overwriting earlier ones). Close the pylon Viewer
first - it holds the camera exclusively.
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
from live_view import LiveView

from lerobot.cameras.utils import make_cameras_from_configs
from rigs import rig_config

# --- CONFIG -----------------------------------------------------------------------------
RIG = "meca500_basler"
CAMERA_KEY = "microscope_cam"
OUT_DIR = Path(__file__).resolve().parents[2] / "assets" / "benches" / RIG / "beads"
PREFIX = "beads"
# ----------------------------------------------------------------------------------------


def main() -> None:
    config = rig_config(RIG)
    camera = make_cameras_from_configs({CAMERA_KEY: config.cameras[CAMERA_KEY]})[CAMERA_KEY]
    try:
        camera.connect()
    except Exception as e:
        sys.exit(f"Could not open {CAMERA_KEY} ({e}). Close the pylon Viewer and any other script.")
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    n = len(list(OUT_DIR.glob(f"{PREFIX}_*.png")))
    view = LiveView("capture_frames")
    print(f"Saving to {OUT_DIR}  (space = save, q = quit)")
    try:
        while not view.closed:
            frame = camera.async_read(timeout_ms=1000)
            view.show(frame, title=f"space = save   q = quit   ({n} saved)", crosshair=True)
            key = view.key()
            if key == "q":
                break
            if key == " ":
                path = OUT_DIR / f"{PREFIX}_{n:03d}.png"
                while path.exists():
                    n += 1
                    path = OUT_DIR / f"{PREFIX}_{n:03d}.png"
                cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                print(f"  saved {path.name}")
                n += 1
    finally:
        camera.disconnect()
        view.close()


if __name__ == "__main__":
    main()
