"""Resolve Windows DirectShow camera indices by stable device name.

OpenCV's integer camera index is only a *position* in the DirectShow enumeration
order. That order changes whenever a USB camera is unplugged, replugged, or the
machine is power-cycled, so a hard-coded index silently starts pointing at the
wrong camera after any of those events (the classic "the microscope moved from
index 3 to index 1 after I replugged it" problem).

This module maps each rig camera to its stable DirectShow *friendly name* and
returns the index that name currently occupies, so the config never has to trust
a raw index:

  * The microscope (``VMS700``) and built-in webcam (``Chicony...``) have unique
    names, so they are always found no matter where they land.
  * The overhead and wrist cameras are the *same model* and both report
    ``UC60 Video``, so Windows cannot tell them apart by name. They are only
    distinguishable by which USB port each sits in. Keep each in its own port:
    the lower enumeration index is the overhead cam, the higher is the wrist cam.

Windows-only (needs ``pygrabber``/DirectShow, which uses the same device
enumerator as OpenCV's ``CAP_DSHOW`` backend, so their ordering matches). On any
other platform, or if the lookup fails, ``resolve_camera_indices`` returns an
empty mapping and callers keep the static indices from the config.
"""

from __future__ import annotations

import logging
import platform

logger = logging.getLogger(__name__)

# DirectShow friendly names as they appear on the Windows rig (confirmed with
# scripts/meca500/check_camera.py and pygrabber). Update these if a camera is
# swapped for a different model.
MICROSCOPE_DSHOW_NAME = "VMS700"
ARM_CAM_DSHOW_NAME = "UC60 Video"  # overhead + wrist are the same model, share this name


def _dshow_device_names() -> list[str]:
    """DirectShow input-device names in OpenCV's index order (position == cv2 index)."""
    from pygrabber.dshow_graph import FilterGraph  # Windows/DirectShow-only import

    return FilterGraph().get_input_devices()


def resolve_camera_indices() -> dict[str, int]:
    """Map rig camera roles to their current OpenCV indices by DirectShow name.

    Returns a dict with any of ``"microscope_cam"``, ``"overhead_cam"``,
    ``"wrist_cam"`` that could be resolved unambiguously. Cameras that are
    missing or ambiguous are omitted (with a warning) so the caller falls back to
    the configured index for those. Returns ``{}`` off Windows or if enumeration
    fails (e.g. pygrabber not installed).
    """
    if platform.system() != "Windows":
        return {}
    try:
        names = _dshow_device_names()
    except Exception as e:  # pygrabber missing or COM/DirectShow failure
        logger.warning(f"Camera name resolution unavailable ({e}); using configured indices.")
        return {}

    resolved: dict[str, int] = {}

    micro = [i for i, n in enumerate(names) if MICROSCOPE_DSHOW_NAME in n]
    if len(micro) == 1:
        resolved["microscope_cam"] = micro[0]
    else:
        logger.warning(
            f"Expected exactly one '{MICROSCOPE_DSHOW_NAME}' camera, found {len(micro)} in {names}; "
            f"leaving microscope_cam at its configured index."
        )

    arm_cams = [i for i, n in enumerate(names) if n == ARM_CAM_DSHOW_NAME]
    if len(arm_cams) >= 2:
        resolved["overhead_cam"], resolved["wrist_cam"] = arm_cams[0], arm_cams[1]
        if len(arm_cams) > 2:
            logger.warning(
                f"Found {len(arm_cams)} '{ARM_CAM_DSHOW_NAME}' cameras; using the first two as "
                f"overhead (idx {arm_cams[0]}) and wrist (idx {arm_cams[1]})."
            )
    else:
        logger.warning(
            f"Expected two '{ARM_CAM_DSHOW_NAME}' cameras, found {len(arm_cams)} in {names}; "
            f"leaving overhead_cam/wrist_cam at their configured indices."
        )

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
