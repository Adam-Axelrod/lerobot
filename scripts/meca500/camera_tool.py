"""One tool for the Meca500 cameras: identify them, see them, tune them, save them.

Run this ON THE WINDOWS LAPTOP (not the Mac) - it's the only place the cameras and
their real drivers exist.

    python scripts/meca500/camera_tool.py               # identify + live preview, tune and save
    python scripts/meca500/camera_tool.py --list        # just the identity table, no windows
    python scripts/meca500/camera_tool.py --verify      # prove autoexposure/autofocus are really off
    python scripts/meca500/camera_tool.py --rig meca500 # the plain 2-camera rig (default: microscope)
    python scripts/meca500/camera_tool.py --rig meca500_microscope_basler  # Basler microscope cam
    python scripts/meca500/camera_tool.py --cameras wrist_cam,overhead_cam

Replaces the old identify_cameras.py / check_camera.py / verify_camera_settings.py.

WHAT IT SHOWS
    Every camera the selected rig defines, opened through the *same* camera classes
    the recording scripts use, at the same resolution/codec/exposure/focus. What you
    see here is what gets recorded - there is no second copy of the setup logic that
    could drift.

    Exposure/focus tuning applies to the USB webcams (OpenCV) only. The microscope is
    a Basler: its exposure and gain live in the camera's own stored user set, so tune
    it in the pylon Viewer and "Save User Set". It still previews here.

TUNING (live, in the preview window)
    1..9        select the active camera
    left/right  exposure  -/+          up/down   focus  -/+
    s           save the current values to camera_settings.json
    r           reload the saved values (discard un-saved changes)
    q / Esc     quit

    Saving writes src/lerobot/robots/meca500/camera_settings.json, which every robot
    config loads in __post_init__ - so recording, teleop and inference all pick the
    new values up with no further edits. The file is git-tracked, so a recorded
    dataset can always be traced back to the settings that produced it.

NOTE ON USB BANDWIDTH
    The two UVC webcams share one bus (which is why the wrist cam uses MJPG), and the
    Basler wants a good chunk of its own. Opening everything at once may fail or
    crawl. A camera that will not open is reported and skipped rather than aborting
    the run - use --cameras to preview a subset if the rig is tight.
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import platform
import sys
import time

import cv2  # type: ignore
import matplotlib.pyplot as plt
import numpy as np

from lerobot.cameras.camera import Camera
from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.robots.meca500.camera_settings import (
    SETTINGS_PATH,
    read_tunables,
    save_camera_settings,
)
from lerobot.robots.meca500_microscope.camera_resolver import (
    dshow_devices,
    instance_token_of,
    resolve_camera_indices,
)
from rigs import RIGS

logger = logging.getLogger(__name__)


EXPOSURE_STEP = 1  # DSHOW exposure is a log2 scale; 1 step ~= one stop
FOCUS_STEP = 5


# --------------------------------------------------------------------------- list


def print_identity_table(rig: str) -> None:
    """Every video device, its stable USB id, and the rig role it resolves to."""
    if platform.system() != "Windows":
        print("(device identity listing is Windows/DirectShow only - skipping)")
        return
    try:
        devices = dshow_devices()
    except Exception as e:
        print(f"Could not enumerate DirectShow devices: {type(e).__name__}: {e}")
        return

    # Ask only about the roles THIS rig addresses by index. Resolving all of them
    # regardless would announce that the VMS700 and the two UC60s are missing whenever the
    # rig does not have them - on meca500_basler, every line of it a false alarm. That is
    # precisely what camera_resolver's `roles` argument exists to prevent.
    cameras = RIGS[rig]().cameras
    index_roles = [role for role, cam in cameras.items() if hasattr(cam, "index_or_path")]
    resolved = resolve_camera_indices(index_roles) if index_roles else {}
    role_by_index = {index: role for role, index in resolved.items()}

    print(f"\n{len(devices)} camera(s) visible to DirectShow (index == OpenCV index):\n")
    for index, (name, path) in enumerate(devices):
        print(f"  [{index}] {name}")
        print(f"        role now : {role_by_index.get(index, '-')}")
        print(f"        usb id   : {instance_token_of(path)}")

    # Serial-addressed cameras (the Basler) never appear in the DirectShow enumeration, so
    # without this the table stays silent about them - actively misleading on a rig whose
    # only camera is one, where it reports "1 camera" and does not mean that one.
    serial_cams = {role: cam for role, cam in cameras.items() if hasattr(cam, "serial_number")}
    if serial_cams:
        print()
        print(f'  {len(serial_cams)} camera(s) addressed by serial (not in DirectShow, nothing to resolve):')
        for role, cam in serial_cams.items():
            print(f'  [sn {cam.serial_number}] {role}')

    # Only index-addressed roles can go UNRESOLVED; flagging a serial-addressed camera
    # would be a warning that is always wrong.
    for role in index_roles:
        if role not in resolved:
            print(f"\n  WARNING: {role} is UNRESOLVED - it falls back to its configured index.")
    print()


# ------------------------------------------------------------------------ preview


def _device_id(cfg) -> str:
    """How this camera is addressed, for display: an OpenCV index or a Basler serial."""
    if hasattr(cfg, "index_or_path"):
        return f"idx {cfg.index_or_path}"
    if hasattr(cfg, "serial_number"):
        return f"sn {cfg.serial_number}"
    return "?"


def is_tunable(camera: Camera) -> bool:
    """Whether exposure/focus can be driven from this tool.

    Only the OpenCV backend exposes the CAP_PROP_* controls this tool pokes. A Basler
    previews fine but keeps its exposure/gain in the camera's own stored user set, so the
    tuning keys and the settings file do not apply to it.
    """
    return isinstance(camera, OpenCVCamera)


def open_cameras(rig: str, roles: list[str]) -> dict[str, Camera]:
    """Connect each requested role through the real camera class the rig config names.

    Goes through `make_cameras_from_configs` rather than constructing a backend directly,
    so this keeps working when a role changes backend (as the microscope did, OpenCV ->
    Basler) instead of silently opening the wrong kind of device.

    Cameras that fail to open are reported and skipped: on this rig that usually
    means USB bandwidth, not a config error, and previewing the rest is still useful.
    """
    config = RIGS[rig]()
    cameras = make_cameras_from_configs({role: config.cameras[role] for role in roles})

    opened: dict[str, Camera] = {}
    for role, camera in cameras.items():
        cam_cfg = config.cameras[role]
        try:
            camera.connect()
            opened[role] = camera
            print(f"  opened {role:15s} {_device_id(cam_cfg)} {cam_cfg.width}x{cam_cfg.height}")
        except Exception as e:
            print(f"  FAILED {role:15s} {_device_id(cam_cfg)}: {type(e).__name__}: {e}")
    return opened


def _set_control(camera: OpenCVCamera, kind: str, value: float) -> None:
    """Push one control to the device using OpenCVCamera's own setter.

    Reusing `_try_set_property` keeps the backend-specific quirks (the DSHOW
    auto-exposure magic values, the unreliable Windows read-back) in exactly one
    place instead of forking a second copy of them here.
    """
    prop = cv2.CAP_PROP_EXPOSURE if kind == "exposure" else cv2.CAP_PROP_FOCUS
    camera._try_set_property(kind, prop, float(value))


def _title(role: str, camera: Camera, active: bool, dirty: bool) -> str:
    cfg = camera.config
    mark = ">> " if active else "   "
    if not is_tunable(camera):
        return f"{mark}{role}  [{_device_id(cfg)}]\nexposure tuned on the camera itself"
    star = " *" if dirty else ""
    focus = "n/a" if cfg.focus is None else cfg.focus
    return f"{mark}{role}  [{_device_id(cfg)}]\nexp={cfg.exposure}  focus={focus}{star}"


def preview(rig: str, roles: list[str], allow_tuning: bool) -> None:
    cameras = open_cameras(rig, roles)
    if not cameras:
        sys.exit("No cameras could be opened. Check USB connections, then re-run with --list.")

    order = list(cameras)
    state = {"active": 0, "dirty": False, "quit": False}

    n = len(order)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), squeeze=False)
    axes = list(axes[0])
    fig.canvas.manager.set_window_title(f"{rig} cameras - {'tune + ' if allow_tuning else ''}preview")
    images: dict[str, object] = {}

    def on_key(event) -> None:
        key = event.key
        if key in ("q", "escape"):
            state["quit"] = True
            return
        if key and key.isdigit() and 1 <= int(key) <= n:
            state["active"] = int(key) - 1
            return
        if not allow_tuning:
            return

        role = order[state["active"]]
        camera = cameras[role]
        cfg = camera.config

        if key in ("left", "right", "up", "down") and not is_tunable(camera):
            print(
                f"  {role} is a {type(camera).__name__} - exposure/gain live in the camera's own "
                f"stored user set. Tune it in the pylon Viewer and 'Save User Set'."
            )
            return

        if key in ("left", "right") and cfg.exposure is not None:
            cfg.exposure += EXPOSURE_STEP if key == "right" else -EXPOSURE_STEP
            _set_control(camera, "exposure", cfg.exposure)
            state["dirty"] = True
        elif key in ("up", "down") and cfg.focus is not None:
            cfg.focus = max(0, cfg.focus + (FOCUS_STEP if key == "up" else -FOCUS_STEP))
            _set_control(camera, "focus", cfg.focus)
            state["dirty"] = True
        elif key == "s":
            # Roles with no tunable fields would be written as empty objects, which reads as
            # "tuned to nothing" rather than "not tuned here". Leave them out entirely.
            tunables = read_tunables({r: cameras[r].config for r in order if is_tunable(cameras[r])})
            save_camera_settings(rig, tunables)
            state["dirty"] = False
            print(f"\nSaved {rig} camera settings to {SETTINGS_PATH}")
            for role_name, values in tunables.items():
                print(f"  {role_name:15s} " + "  ".join(f"{k}={v}" for k, v in values.items()))
            print("  Recording/teleop/inference will now use these.\n")
        elif key == "r":
            fresh = RIGS[rig]().cameras
            for role_name in order:
                if not is_tunable(cameras[role_name]):
                    continue
                saved = fresh[role_name]
                cameras[role_name].config.exposure = saved.exposure
                cameras[role_name].config.focus = saved.focus
                if saved.exposure is not None:
                    _set_control(cameras[role_name], "exposure", saved.exposure)
                if saved.focus is not None:
                    _set_control(cameras[role_name], "focus", saved.focus)
            state["dirty"] = False
            print("Reloaded saved settings.")

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _e: state.update(quit=True))

    if allow_tuning:
        print(
            "\n  1..9 select camera | left/right exposure | up/down focus | "
            "s save | r reload | q quit\n"
        )
    else:
        print("\n  1..9 select camera | q quit\n")

    plt.ion()
    while not state["quit"]:
        for i, role in enumerate(order):
            camera = cameras[role]
            try:
                frame = camera.async_read(timeout_ms=500)
            except Exception:
                continue
            if images.get(role) is None:
                images[role] = axes[i].imshow(frame)
                axes[i].axis("off")
            else:
                images[role].set_data(frame)
            axes[i].set_title(
                _title(role, camera, active=(i == state["active"]), dirty=state["dirty"]),
                fontsize=10,
            )
        plt.pause(0.001)

    if state["dirty"]:
        print("\nWARNING: you changed settings but did not press 's' - they were NOT saved.\n")

    for camera in cameras.values():
        with contextlib.suppress(Exception):
            camera.disconnect()
    plt.close(fig)


# ------------------------------------------------------------------------- verify


def sharpness(frame: np.ndarray) -> float:
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    return float(cv2.Laplacian(grey, cv2.CV_64F).var())


def verify(rig: str, roles: list[str]) -> None:
    """Sweep exposure and focus and report what the picture actually does.

    Read-back cannot be trusted on Windows (DSHOW reports sentinel values and set()
    returns success even when ignored), so the only real evidence that the auto
    loops are off is that the image responds to manual values and stays where it is
    put. Brightness should climb across the exposure sweep; sharpness should peak
    somewhere in the focus sweep.
    """
    cameras = open_cameras(rig, roles)
    if not cameras:
        sys.exit("No cameras could be opened.")

    for role, camera in cameras.items():
        cfg = camera.config
        print(f"\n=== {role} ({_device_id(cfg)}) ===")

        if not is_tunable(camera):
            print("  exposure/gain are stored on the camera itself - nothing to sweep here.")
            print("  Verify it in the pylon Viewer instead.")
            with contextlib.suppress(Exception):
                camera.disconnect()
            continue

        if cfg.exposure is not None:
            print("  exposure sweep (brightness should climb):")
            for value in range(cfg.exposure - 3, cfg.exposure + 4):
                _set_control(camera, "exposure", value)
                time.sleep(0.35)
                try:
                    frame = camera.async_read(timeout_ms=800)
                except Exception:
                    print(f"    exp={value:>4}  <no frame>")
                    continue
                print(f"    exp={value:>4}  mean brightness={frame.mean():6.1f}")
            _set_control(camera, "exposure", cfg.exposure)

        if cfg.focus is not None:
            print("  focus sweep (sharpness should peak, not stay flat):")
            for value in range(0, 256, 51):
                _set_control(camera, "focus", value)
                time.sleep(0.45)
                try:
                    frame = camera.async_read(timeout_ms=800)
                except Exception:
                    print(f"    focus={value:>4}  <no frame>")
                    continue
                print(f"    focus={value:>4}  sharpness={sharpness(frame):8.1f}")
            _set_control(camera, "focus", cfg.focus)
        else:
            print("  focus: fixed lens (no software focus) - nothing to sweep.")

        with contextlib.suppress(Exception):
            camera.disconnect()
    print()


# --------------------------------------------------------------------------- main


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rig", choices=sorted(RIGS), default="meca500_microscope")
    parser.add_argument("--list", action="store_true", help="print the identity table and exit")
    parser.add_argument("--verify", action="store_true", help="sweep exposure/focus instead of previewing")
    parser.add_argument("--cameras", help="comma-separated roles (default: all cameras on the rig)")
    parser.add_argument("--no-tune", action="store_true", help="preview read-only (no live editing)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    available = list(RIGS[args.rig]().cameras)
    if args.cameras:
        roles = [r.strip() for r in args.cameras.split(",") if r.strip()]
        unknown = [r for r in roles if r not in available]
        if unknown:
            sys.exit(f"Unknown camera role(s) {unknown} for rig {args.rig}. Available: {available}")
    else:
        roles = available

    print_identity_table(args.rig)
    if args.list:
        return

    if args.verify:
        verify(args.rig, roles)
    else:
        preview(args.rig, roles, allow_tuning=not args.no_tune)


if __name__ == "__main__":
    main()
