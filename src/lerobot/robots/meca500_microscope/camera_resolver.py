"""Resolve Windows DirectShow camera indices by stable USB device identity.

OpenCV's integer camera index is only a *position* in the DirectShow enumeration
order. That order changes whenever a USB camera is unplugged, replugged, or the
machine is power-cycled, so a hard-coded index silently starts pointing at the
wrong camera after any of those events (the classic "the microscope moved from
index 3 to index 1 after I replugged it" problem).

Each rig camera is therefore matched to a *stable* property rather than a position:

  * The microscope (``VMS700``) has a unique DirectShow friendly name, so the name
    alone identifies it wherever it lands.
  * The overhead and wrist cameras are the *same model* and both report the
    friendly name ``UC60 Video``, so names cannot tell them apart. They are
    distinguished by USB device identity instead (their ``DevicePath``):

      - One of the two reports a real USB serial number, so Windows builds its
        device instance path from that serial and the path stays the same in any
        port and across reboots. That camera is pinned by ``PINNED_ARM_CAM_USB_ID``.
      - The other reports no serial, so its instance path is derived from the
        hub/port and *does* change when it is moved. It never needs to be matched
        directly: it is simply the remaining ``UC60 Video`` device, by elimination.

    The result is independent of both USB port and enumeration order.

If the arm cameras cannot be identified this way, their roles are left UNRESOLVED
rather than guessed from enumeration order. Guessing is what silently swapped
overhead and wrist after a replug, and a dataset recorded with two views
transposed is far more expensive than a loud warning.

Windows-only (needs ``pygrabber``/DirectShow, which uses the same device
enumerator as OpenCV's ``CAP_DSHOW`` backend, so their ordering matches). On any
other platform, or if enumeration fails, ``resolve_camera_indices`` returns an
empty mapping and callers keep the static indices from the config.

To re-derive the constants below (after swapping a camera for a different unit, or
to confirm which physical camera is which):

    python scripts/meca500/camera_tool.py --list
"""

from __future__ import annotations

import logging
import platform
import re

logger = logging.getLogger(__name__)

# The device-instance token inside a DevicePath, e.g.
#   \\?\usb#vid_0bda&pid_5805&mi_00#6&5e564cb&0&0000#{65e8773d-...}\global
#                                    ^^^^^^^^^^^^^^^^
_INSTANCE_TOKEN = re.compile(r"#(6&[0-9a-f]+&[0-9]+&[0-9]+)#", re.IGNORECASE)

# DirectShow friendly names as they appear on the Windows rig (confirmed with
# scripts/meca500/camera_tool.py --list). Update these if a camera is swapped for a
# different model.
MICROSCOPE_DSHOW_NAME = "VMS700"
ARM_CAM_DSHOW_NAME = "UC60 Video"  # overhead + wrist are the same model, share this name

# Stable USB device-instance token of the one arm camera that reports a hardware
# serial (USB\VID_0BDA&PID_5805\200901010001). Matched case-insensitively as a
# substring of the DirectShow DevicePath. The other UC60 reports no serial and is
# taken by elimination, so only this one needs pinning.
PINNED_ARM_CAM_USB_ID = "6&5e564cb&0&0000"

# Which physical role the pinned camera fills; the other UC60 gets the other role.
# VERIFY before trusting a recording: `python scripts/meca500/camera_tool.py` shows
# each camera live, titled with the role it resolved to. If they are transposed,
# flipping this constant is the whole fix.
PINNED_ARM_CAM_ROLE = "overhead_cam"  # -> the other UC60 is wrist_cam

_ARM_ROLES = ("overhead_cam", "wrist_cam")


def dshow_devices() -> list[tuple[str, str]]:
    """``(friendly_name, device_path)`` per video input, in OpenCV index order.

    ``device_path`` is the USB device instance path DirectShow exposes on the
    device moniker's property bag; it is ``""`` for devices that do not expose one.
    """
    from comtypes import GUID, client  # Windows/COM-only imports
    from comtypes.persist import IPropertyBag
    from pygrabber.dshow_core import ICreateDevEnum
    from pygrabber.dshow_ids import DeviceCategories, clsids

    device_enum = client.CreateObject(clsids.CLSID_SystemDeviceEnum, interface=ICreateDevEnum)
    enumerator = device_enum.CreateClassEnumerator(GUID(DeviceCategories.VideoInputDevice), dwFlags=0)

    devices: list[tuple[str, str]] = []
    try:
        moniker, count = enumerator.Next(1)
    except ValueError:  # no video input devices at all
        return devices

    while count > 0:
        bag = moniker.BindToStorage(0, 0, IPropertyBag._iid_).QueryInterface(IPropertyBag)
        name = bag.Read("FriendlyName", pErrorLog=None)
        try:
            path = bag.Read("DevicePath", pErrorLog=None)
        except Exception:  # virtual/legacy devices may not expose one
            path = ""
        devices.append((name, path))
        moniker, count = enumerator.Next(1)
    return devices


def instance_token_of(device_path: str) -> str:
    """The stable device-instance token from a DevicePath, or the raw path if unmatched.

    This is the value to paste into :data:`PINNED_ARM_CAM_USB_ID` when a camera is
    replaced; ``scripts/meca500/camera_tool.py --list`` prints it per device.
    """
    match = _INSTANCE_TOKEN.search(device_path or "")
    return match.group(1) if match else (device_path or "<none>")


def _resolve_arm_cams(devices: list[tuple[str, str]], resolved: dict[str, int]) -> None:
    """Assign overhead/wrist by USB identity, or assign neither.

    Deliberately never falls back to enumeration order: a wrong-but-plausible
    mapping here is invisible until the data is already recorded.
    """
    if PINNED_ARM_CAM_ROLE not in _ARM_ROLES:
        raise ValueError(f"PINNED_ARM_CAM_ROLE must be one of {_ARM_ROLES}, got {PINNED_ARM_CAM_ROLE!r}")
    other_role = next(role for role in _ARM_ROLES if role != PINNED_ARM_CAM_ROLE)

    arm_cams = [(i, path) for i, (name, path) in enumerate(devices) if name == ARM_CAM_DSHOW_NAME]
    if len(arm_cams) != 2:
        logger.warning(
            f"Expected exactly two {ARM_CAM_DSHOW_NAME!r} cameras, found {len(arm_cams)}; leaving "
            f"overhead_cam/wrist_cam at their configured indices. Check the USB connections, then "
            f"re-run scripts/meca500/camera_tool.py --list."
        )
        return

    needle = PINNED_ARM_CAM_USB_ID.lower()
    pinned = [i for i, path in arm_cams if needle in (path or "").lower()]
    if len(pinned) != 1:
        logger.warning(
            f"Could not identify the arm cameras by USB device id: expected exactly one "
            f"{ARM_CAM_DSHOW_NAME!r} whose DevicePath contains {PINNED_ARM_CAM_USB_ID!r}, found "
            f"{len(pinned)}. NOT guessing from enumeration order, because that is how overhead and "
            f"wrist get silently transposed. overhead_cam/wrist_cam keep their configured indices - "
            f"run scripts/meca500/camera_tool.py --list and update PINNED_ARM_CAM_USB_ID before recording."
        )
        return

    resolved[PINNED_ARM_CAM_ROLE] = pinned[0]
    resolved[other_role] = next(i for i, _ in arm_cams if i != pinned[0])


def resolve_camera_indices() -> dict[str, int]:
    """Map rig camera roles to their current OpenCV indices.

    Returns a dict with any of ``"microscope_cam"``, ``"overhead_cam"``,
    ``"wrist_cam"`` that could be resolved unambiguously. Roles that are missing or
    ambiguous are omitted (with a warning) so the caller falls back to the
    configured index for those. Returns ``{}`` off Windows or if enumeration fails
    (e.g. pygrabber not installed).
    """
    if platform.system() != "Windows":
        return {}
    try:
        devices = dshow_devices()
    except Exception as e:  # pygrabber/comtypes missing or COM/DirectShow failure
        logger.warning(f"Camera identity resolution unavailable ({e}); using configured indices.")
        return {}

    names = [name for name, _ in devices]
    resolved: dict[str, int] = {}

    micro = [i for i, n in enumerate(names) if MICROSCOPE_DSHOW_NAME in n]
    if len(micro) == 1:
        resolved["microscope_cam"] = micro[0]
    else:
        logger.warning(
            f"Expected exactly one {MICROSCOPE_DSHOW_NAME!r} camera, found {len(micro)} in {names}; "
            f"leaving microscope_cam at its configured index."
        )

    _resolve_arm_cams(devices, resolved)
    return resolved


def apply_resolved_indices(cameras: dict) -> dict:
    """Rewrite each camera config's ``index_or_path`` to its current live index.

    ``cameras`` is a robot config's ``cameras`` dict ({role: OpenCVCameraConfig}).
    Mutates it in place and returns it. Roles that cannot be resolved keep their
    configured index. Every change is logged so the live mapping is visible at
    startup.
    """
    for role, index in resolve_camera_indices().items():
        cam = cameras.get(role)
        if cam is None:
            continue
        if cam.index_or_path != index:
            logger.info(f"{role}: resolved to camera index {index} (was {cam.index_or_path}).")
        cam.index_or_path = index
    return cameras
