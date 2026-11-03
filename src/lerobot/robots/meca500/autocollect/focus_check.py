"""Interlock: refuse to drive the arm to the dish unless the microscope focus is where it was
when the bench was calibrated.

The focus drive (a Thorlabs K-Cube) is always set so the dish bottom is sharp, so its position
tracks the dish surface height. If it reads differently from ``scope.focus_mm`` in bench.yaml, the
dish has been raised, lowered or swapped, or someone refocused, and the calibrated dish height can
no longer be trusted. On 2026-11-03 an acrylic holder raised the dish ~22 mm; record_auto still
planned to the old height and bent the pipette on the dish. Nothing checked the dish height.

``scope.focus_mm`` must only change together with the dish height and calibration (Claude updates
them together after a refocus).

Reading the position connects to the K-Cube and polls it; it never enables, homes or moves the
motor. Kinesis must be closed, because only one program can hold the controller.
"""

from __future__ import annotations

import time

KINESIS_DIR = r"C:\Program Files\Thorlabs\Kinesis"
DEFAULT_TOLERANCE_MM = 0.05


class FocusMismatchError(RuntimeError):
    """The focus is not where the bench calibration expects it, or could not be read."""


def read_kcube_stepper_mm(serial: str) -> float:
    """Position (mm) of a KST101/KST201 K-Cube stepper. Read-only."""
    import clr  # pythonnet

    for dll in ("DeviceManagerCLI", "GenericMotorCLI", "KCube.StepperMotorCLI"):
        clr.AddReference(rf"{KINESIS_DIR}\Thorlabs.MotionControl.{dll}.dll")
    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI  # type: ignore
    from Thorlabs.MotionControl.KCube.StepperMotorCLI import KCubeStepper  # type: ignore

    DeviceManagerCLI.BuildDeviceList()
    if serial not in list(DeviceManagerCLI.GetDeviceList()):
        raise FocusMismatchError(
            f"Focus K-Cube {serial} not found. Is it powered and plugged in by USB, and is Kinesis closed?"
        )
    dev = KCubeStepper.CreateKCubeStepper(serial)
    dev.Connect(serial)
    try:
        if not dev.IsSettingsInitialized():
            dev.WaitForSettingsInitialized(5000)
        dev.StartPolling(100)
        time.sleep(0.4)
        dev.LoadMotorConfiguration(serial)  # for real units (mm); does not move the motor
        pos = float(str(dev.Position))
        dev.StopPolling()
        return pos
    finally:
        dev.Disconnect(False)


def check_focus(scope: dict, tolerance_mm: float = DEFAULT_TOLERANCE_MM) -> float:
    """Raise FocusMismatchError unless the focus K-Cube reads ``scope['focus_mm']``.

    ``scope`` is bench.yaml's ``scope`` section. Returns the position read.
    """
    expected = scope.get("focus_mm")
    serial = scope.get("focus_kcube_serial")
    if expected is None or serial is None:
        raise FocusMismatchError("bench.yaml scope has no focus_mm / focus_kcube_serial to check against.")
    try:
        pos = read_kcube_stepper_mm(str(serial))
    except FocusMismatchError:
        raise
    except Exception as e:  # pythonnet or Kinesis missing, controller held by Kinesis, ...
        raise FocusMismatchError(f"Could not read the focus K-Cube {serial} ({type(e).__name__}: {e}).") from e
    if abs(pos - float(expected)) > tolerance_mm:
        raise FocusMismatchError(
            f"The microscope focus is at {pos:.4f} mm but the bench was calibrated at {float(expected):.4f} mm "
            f"(tolerance {tolerance_mm} mm). The dish height has probably changed (new holder, reseated or "
            "swapped dish, or a refocus), so the calibrated dish height is wrong and the arm could drive the "
            "pipette into the dish. Do not override this: the dish height, model and calibration must be "
            "updated first (ask Claude)."
        )
    return pos
