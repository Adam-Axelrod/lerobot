"""Verify autofocus/auto-exposure locking on the Windows machine wired to the Meca500.

Run this ON THE WINDOWS LAPTOP (not the Mac) — it's the only place the cameras and
their real drivers exist:

    uv run python scripts/meca500/verify_camera_settings.py
    # (plain `python scripts/meca500/verify_camera_settings.py` also works)

It does three things per camera, using the exact `connect()` code path the robot uses:
  1. Prints which OpenCV backend is actually in use (MSMF vs DSHOW vs ...). The correct
     CAP_PROP_AUTO_EXPOSURE magic value depends on this.
  2. Reads back the four properties after connect, so you can see what the driver stored.
     NOTE: on Windows, get() is unreliable and may not echo set() — a mismatch here does
     NOT necessarily mean the lock failed. Treat it as informational.
  3. Runs the real functional tests for BOTH controls over a few seconds:
       - brightness (mean pixel value)      -> with auto-exposure OFF it stays ~flat even as
         you change the lighting; if it tracks the lighting, the exposure lock did NOT take.
       - sharpness (variance of Laplacian)  -> with autofocus OFF it stays ~flat even as you
         move an object toward/away from the lens; if it drifts, autofocus is still hunting.
     Read-back (step 2) can't be trusted on Windows, so these metrics are the real check.
"""

import statistics
import time

import cv2  # type: ignore
import numpy as np

from lerobot.cameras.opencv import OpenCVCamera
from lerobot.robots.meca500.config_meca500 import Meca500Config

# Human-readable names for the properties we lock.
PROP_NAMES = {
    "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "autofocus": cv2.CAP_PROP_AUTOFOCUS,
    "focus": cv2.CAP_PROP_FOCUS,
}

SAMPLE_SECONDS = 6.0


def backend_name(cap: cv2.VideoCapture) -> str:
    try:
        return cap.getBackendName()
    except Exception:
        return f"id={int(cap.get(cv2.CAP_PROP_BACKEND))}"


def check_camera(name: str, cam: OpenCVCamera) -> None:
    print(f"\n=== {name} (index {cam.index_or_path}) ===")
    cam.connect(warmup=True)
    cap = cam.videocapture
    assert cap is not None

    print(f"backend in use : {backend_name(cap)}")

    cfg = cam.config
    print("requested -> driver read-back (read-back may be unreliable on Windows):")
    for label, prop in PROP_NAMES.items():
        requested = {
            "auto_exposure": cfg.autoexposure,
            "exposure": cfg.exposure,
            "autofocus": cfg.autofocus,
            "focus": cfg.focus,
        }[label]
        print(f"  {label:<13} requested={requested!s:<7} read_back={cap.get(prop)}")

    # Functional test: are exposure AND focus actually locked? We sample two metrics
    # per frame over the same window:
    #   - mean pixel value      -> brightness, tracks auto-EXPOSURE
    #   - variance of Laplacian -> sharpness, tracks auto-FOCUS (autofocus hunting
    #                              makes a static scene drift in/out of focus)
    # With both locked and a static scene, both metrics should stay ~flat. To exercise
    # them, change the lighting (for exposure) and move an object toward/away from the
    # lens (for focus) during the sampling window.
    print(
        f"\nsampling brightness + sharpness for {SAMPLE_SECONDS:.0f}s — change the lighting AND "
        f"move an object toward/away from the lens to test..."
    )
    brightness = []
    sharpness = []
    t0 = time.time()
    while time.time() - t0 < SAMPLE_SECONDS:
        frame = cam.async_read(timeout_ms=1000)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        brightness.append(float(np.mean(gray)))
        sharpness.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
        time.sleep(0.1)

    def report(label: str, values: list[float], threshold_pct: float, still_on: str) -> None:
        lo, hi = min(values), max(values)
        spread = hi - lo
        pct = 100.0 * spread / max(statistics.mean(values), 1e-6)
        print(f"  {label}: {lo:.1f}..{hi:.1f}  (spread {spread:.1f}, {pct:.1f}% of mean)")
        if pct < threshold_pct:
            print(f"    -> LOOKS LOCKED: {label} barely moved. ✅")
        else:
            print(f"    -> NOT LOCKED: {label} drifted; {still_on} ❌")

    report("brightness", brightness, 5.0, "auto-exposure is still active.")
    report("sharpness ", sharpness, 15.0, "autofocus is still hunting.")

    cam.disconnect()


def main() -> None:
    cfg = Meca500Config()
    for name, cam_cfg in cfg.cameras.items():
        check_camera(name, OpenCVCamera(cam_cfg))


if __name__ == "__main__":
    main()
