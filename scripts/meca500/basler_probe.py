"""Standalone connectivity check for the Basler microscope camera.

Why this exists: when the Basler will not stream, the cause is almost always *outside*
lerobot — the pylon Viewer still holding the device open, a missing USB kernel driver, a
camera left in triggered mode by a previous session. Debugging that through
`record_microscope.py` means the robot arm, three cameras and a teleop loop all have to work
before you see the one error you care about.

This script imports nothing from lerobot. If it works, the camera is fine and any remaining
problem is in the integration; if it fails, the error is the whole story.

    python scripts/meca500/basler_probe.py --list                   # what is connected
    python scripts/meca500/basler_probe.py --serial 40012345        # open it, measure fps
    python scripts/meca500/basler_probe.py --serial 40012345 --save frame.png

Copy the serial number from `--list` into MICROSCOPE_CAMERA_SERIAL in
src/lerobot/robots/meca500_microscope/basler_microscope_camera.py.

Requires the pylon Camera Software Suite (for the drivers) plus `pip install pypylon`. With
no camera to hand, set PYLON_CAMEMU=1 to make Basler's emulator show up as real devices.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    from pypylon import genicam, pylon  # type: ignore
except ImportError:
    sys.exit(
        "pypylon is not installed. Install it with:\n"
        "    uv pip install 'pypylon>=26.6,<27'\n"
        "and install the pylon Camera Software Suite for the USB3/GigE drivers."
    )

NOTHING_FOUND_HINT = """
No Basler devices found. In rough order of likelihood:
  1. The pylon Viewer (or another Python process) still has the camera open - it takes an
     exclusive lock. Close it and try again.
  2. The pylon Camera Software Suite is not installed. The pip wheel ships user-space
     runtimes only; USB3 cameras need the kernel driver from the installer to enumerate at
     all. Run the pylon USB Configurator to check.
  3. The camera is not powered / the cable is not seated. It should appear in Device Manager.
  4. For GigE: Windows Firewall is blocking device discovery, or the NIC is on a different
     subnet from the camera.
Set PYLON_CAMEMU=1 to test this script against Basler's camera emulator instead.
"""


def list_devices() -> int:
    """Print every Basler device the transport layer can see."""
    devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    if not devices:
        print(NOTHING_FOUND_HINT)
        return 1

    print(f"\n{len(devices)} Basler device(s) found:\n")
    for device in devices:
        # Unset DeviceInfo properties return a sentinel string rather than raising, so the
        # availability predicate is the only way to tell "unset" from a real value.
        user_name = device.GetUserDefinedName() if device.IsUserDefinedNameAvailable() else "-"
        print(f"  serial       : {device.GetSerialNumber()}")
        print(f"  model        : {device.GetModelName()}")
        print(f"  friendly name: {device.GetFriendlyName()}")
        print(f"  device class : {device.GetDeviceClass()}")
        print(f"  user name    : {user_name}")
        print()
    print("Copy the serial into MICROSCOPE_CAMERA_SERIAL in basler_microscope_camera.py.\n")
    return 0


def open_by_serial(serial: str):
    """Open the camera with this serial, or exit with the serials that *were* found."""
    tlf = pylon.TlFactory.GetInstance()
    devices = list(tlf.EnumerateDevices())

    matches = [d for d in devices if d.GetSerialNumber() == serial]
    if not matches:
        if not devices:
            print(NOTHING_FOUND_HINT)
        else:
            found = ", ".join(f"{d.GetSerialNumber()} ({d.GetModelName()})" for d in devices)
            print(f"\nNo device with serial {serial!r}. Found: {found}\n")
        sys.exit(1)

    camera = pylon.InstantCamera(tlf.CreateDevice(matches[0]))
    try:
        camera.Open()
    except genicam.GenericException as e:
        sys.exit(
            f"\nFound serial {serial} but could not open it: {e}\n"
            f"It is most likely still open in the pylon Viewer or another process.\n"
        )
    return camera


def describe(camera) -> None:
    """Print the geometry and capability summary that decides the robot config values."""
    sfnc2 = camera.GetSfncVersion() >= pylon.Sfnc_2_0_0
    fps_node = camera.ResultingAcquisitionFrameRate if sfnc2 else camera.ResultingFrameRateAbs

    width, height = int(camera.Width.Value), int(camera.Height.Value)
    max_width, max_height = int(camera.Width.Max), int(camera.Height.Max)

    print("\n--- camera ---")
    print(f"  model          : {camera.GetDeviceInfo().GetModelName()}")
    print(f"  SFNC version   : {'2.x (USB3 / ace 2)' if sfnc2 else '1.x (GigE ace Classic)'}")
    print(f"  current ROI    : {width}x{height}")
    print(f"  sensor max     : {max_width}x{max_height}")
    print(f"  pixel format   : {camera.PixelFormat.GetValueOrDefault('unknown')}")
    print(f"  resulting fps  : {fps_node.GetValueOrDefault(float('nan')):.2f}")

    # The aspect ratio is the value most likely to be set wrong in the robot config: the
    # BaslerCamera downscales the full sensor, so a mismatched output shape squashes the image.
    ratio = max_width / max_height if max_height else float("nan")
    print(f"  sensor aspect  : {ratio:.3f} ({max_width}:{max_height})")
    print(f"  -> pick width/height with this ratio, e.g. 640x{round(640 / ratio)}")


def measure_fps(camera, frames: int) -> None:
    """Grab a burst of frames and report the rate actually achieved."""
    print(f"\n--- grabbing {frames} frames ---")
    camera.MaxNumBuffer.Value = 5
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    grabbed, failed = 0, 0
    shape = None
    start = time.perf_counter()
    try:
        while grabbed + failed < frames:
            with camera.RetrieveResult(2000, pylon.TimeoutHandling_Return) as result:
                if not result.IsValid():
                    print("  timed out waiting for a frame (is the camera in triggered mode?)")
                    break
                if result.GrabSucceeded():
                    grabbed += 1
                    shape = result.Array.shape
                else:
                    failed += 1
                    print(f"  grab error {result.ErrorCode:#x}: {result.ErrorDescription}")
    finally:
        camera.StopGrabbing()

    elapsed = time.perf_counter() - start
    if grabbed and elapsed > 0:
        print(f"  {grabbed} frames in {elapsed:.2f}s -> {grabbed / elapsed:.1f} fps")
        print(f"  raw frame shape: {shape} (2-D means a mono sensor; lerobot converts it to RGB)")
    if failed:
        print(f"  {failed} failed grabs - on GigE this is usually packet loss (MTU / inter-packet delay)")


def save_frame(camera, path: str) -> None:
    """Grab one frame, convert it to RGB the same way BaslerCamera does, and write a PNG."""
    try:
        import cv2  # noqa: PLC0415  - optional, only needed for --save
    except ImportError:
        sys.exit("--save needs opencv: uv pip install opencv-python")

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_RGB8packed  # plain attribute, not .Value
    converter.OutputBitAlignment.Value = pylon.OutputBitAlignment_MsbAligned

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    try:
        with camera.RetrieveResult(5000, pylon.TimeoutHandling_Return) as result:
            if not result.IsValid() or not result.GrabSucceeded():
                sys.exit("Could not grab a frame to save.")
            rgb = converter.ConvertToArray(result)
    finally:
        camera.StopGrabbing()

    # cv2.imwrite expects BGR; converting here keeps the PNG's colours true.
    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    print(f"\nSaved {rgb.shape} frame to {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Probe the Basler microscope camera without going through lerobot.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--list", action="store_true", help="list every connected Basler device")
    parser.add_argument("--serial", help="serial number of the camera to open")
    parser.add_argument("--frames", type=int, default=100, help="frames to grab when measuring fps")
    parser.add_argument("--save", metavar="PATH", help="save one frame to this PNG path")
    args = parser.parse_args()

    if args.list or not args.serial:
        code = list_devices()
        if not args.serial:
            return code

    camera = open_by_serial(args.serial)
    try:
        describe(camera)
        # Free-running, or the grab below waits forever for a trigger that never comes.
        camera.TriggerMode.TrySetValue("Off")
        measure_fps(camera, args.frames)
        if args.save:
            save_frame(camera, args.save)
    finally:
        # Open once, close once: repeated open/close cycles are a known pypylon failure mode.
        if camera.IsGrabbing():
            camera.StopGrabbing()
        if camera.IsOpen():
            camera.Close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
