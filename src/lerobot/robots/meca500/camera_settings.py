"""Per-rig camera tuning (exposure/focus), persisted next to the robot configs.

Why this exists: the tuned exposure/focus values used to be copied into every
recording script's CONFIG block, into the standalone camera scripts' presets, and into
robot config defaults. Four copies drift, and a drifted copy is invisible until the
data is already recorded. This module makes the JSON file beside it the single
place those values live, and every robot config loads it automatically, so tuning
once applies to recording, teleop and inference alike.

Scope is deliberately narrow. Only the *tunable* controls live in the file:

    autoexposure, exposure, autofocus, focus

Structural settings (resolution, fps, fourcc, backend, index) stay on the dataclass
in ``config_meca500.py`` / ``config_meca500_microscope.py``, where they are
documented alongside the USB-bandwidth reasoning that constrains them. Splitting it
this way means no value is defined in two places, so there is nothing to drift.

The file is git-tracked on purpose. Camera settings change what the pixels look
like, and ``LeRobotDataset`` does not record them, so keeping them in version
control is the only way to trace a recorded dataset back to the settings that
produced it.

Overrides are applied with :func:`dataclasses.replace`, which re-runs
``OpenCVCameraConfig.__post_init__`` — so an invalid combination (``focus`` set
while ``autofocus`` is on, say) raises here rather than being silently accepted, as
it would be by assigning the attribute directly.

Edit via ``python scripts/meca500/camera_tool.py`` rather than by hand.
"""

from __future__ import annotations

import dataclasses
import json
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

SETTINGS_PATH = Path(__file__).parent / "camera_settings.json"

# Only these may appear in the JSON. Anything else is a mistake worth failing on:
# structural settings belong on the dataclass, and a typo would otherwise be a
# silent no-op.
TUNABLE_KEYS = frozenset({"autoexposure", "exposure", "autofocus", "focus"})


def load_settings(path: Path | None = None) -> dict[str, dict[str, dict[str, Any]]]:
    """Read the whole settings file as ``{rig: {role: {setting: value}}}``.

    Returns ``{}`` if the file does not exist (a warning is logged - the file is
    git-tracked, so a missing one means something is off with the checkout).
    Keys starting with ``_`` are treated as comments and skipped.
    """
    path = path or SETTINGS_PATH
    if not path.exists():
        logger.warning(
            f"Camera settings file not found at {path}; using the defaults baked into the robot "
            f"config. Tuned exposure/focus will NOT be applied. Run "
            f"'python scripts/meca500/camera_tool.py' and press 's' to create it."
        )
        return {}

    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise ValueError(f"Camera settings file {path} is not valid JSON: {e}") from e

    if not isinstance(raw, dict):
        raise ValueError(f"Camera settings file {path} must contain a JSON object, got {type(raw).__name__}.")

    return {rig: cfg for rig, cfg in raw.items() if not rig.startswith("_")}


def apply_camera_settings(rig: str, cameras: dict, path: Path | None = None) -> dict:
    """Apply the stored tuning for ``rig`` onto a robot config's ``cameras`` dict.

    ``cameras`` maps role -> camera config; it is mutated in place (each tuned entry
    is replaced with a validated copy) and returned. Roles with no stored settings
    keep their dataclass defaults.

    Raises:
        ValueError: if the file contains an unknown setting key, or a value the
            camera config rejects (e.g. ``focus`` without ``autofocus=False``).
    """
    if not cameras:
        # A deliberately camera-less config (e.g. the homing pre-flight in
        # microscope_inference.py, or reset.py, which pass cameras={} to skip
        # OpenCV startup). There is nothing to apply and no drift to detect, so
        # stay quiet: warning per stored role here would be a false positive on
        # every such run, and this check only earns its keep if it stays rare.
        logger.debug(f"No cameras on this {rig!r} config; skipping camera settings.")
        return cameras

    settings = load_settings(path)
    rig_settings = settings.get(rig)
    if not rig_settings:
        if settings:
            logger.debug(f"No camera settings stored for rig {rig!r}; using dataclass defaults.")
        return cameras

    for role, overrides in rig_settings.items():
        cam = cameras.get(role)
        if cam is None:
            # Loud: a stale role name means the file no longer matches the rig, and
            # the tuning the operator thinks is applied silently would not be.
            logger.warning(
                f"Camera settings list role {role!r} for rig {rig!r}, but that rig has no such "
                f"camera (has: {sorted(cameras)}). Ignoring it - update "
                f"{(path or SETTINGS_PATH).name} or the robot config."
            )
            continue

        unknown = set(overrides) - TUNABLE_KEYS
        if unknown:
            raise ValueError(
                f"Camera settings for {rig}.{role} contain unsupported key(s) {sorted(unknown)}. "
                f"Only {sorted(TUNABLE_KEYS)} may be tuned here; structural settings "
                f"(resolution, fps, fourcc, backend) belong on the robot config dataclass."
            )

        field_names = {f.name for f in dataclasses.fields(cam)}
        missing = set(overrides) - field_names
        if missing:
            raise ValueError(
                f"Camera settings for {rig}.{role} set {sorted(missing)}, which "
                f"{type(cam).__name__} does not support."
            )

        if not overrides:
            continue

        # replace() re-runs __post_init__, so invalid combinations raise here
        # instead of being silently accepted by attribute assignment.
        cameras[role] = dataclasses.replace(cam, **overrides)
        applied = ", ".join(f"{k}={overrides[k]}" for k in sorted(overrides))
        logger.info(f"{rig}.{role}: applied tuned camera settings ({applied}).")

    return cameras


def read_tunables(cameras: dict) -> dict[str, dict[str, Any]]:
    """Current tunable values per role, in the shape the settings file stores."""
    out: dict[str, dict[str, Any]] = {}
    for role, cam in cameras.items():
        field_names = {f.name for f in dataclasses.fields(cam)}
        out[role] = {key: getattr(cam, key) for key in sorted(TUNABLE_KEYS & field_names)}
    return out


def save_camera_settings(
    rig: str, overrides: dict[str, dict[str, Any]], path: Path | None = None
) -> Path:
    """Write ``overrides`` for ``rig`` into the settings file, preserving other rigs.

    Returns the path written. Validates keys before touching the file so a bad call
    cannot leave a half-written config behind.
    """
    path = path or SETTINGS_PATH

    for role, values in overrides.items():
        unknown = set(values) - TUNABLE_KEYS
        if unknown:
            raise ValueError(
                f"Refusing to save {rig}.{role}: unsupported key(s) {sorted(unknown)}. "
                f"Only {sorted(TUNABLE_KEYS)} are tunable."
            )

    raw: dict[str, Any] = {}
    if path.exists():
        raw = json.loads(path.read_text(encoding="utf-8"))

    raw.setdefault(
        "_comment",
        "Tuned camera exposure/focus per rig. Edit with "
        "'python scripts/meca500/camera_tool.py'. Structural settings "
        "(resolution/fps/fourcc/backend) live on the robot config dataclasses.",
    )
    raw[rig] = overrides

    path.write_text(json.dumps(raw, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    logger.info(f"Saved camera settings for {rig} to {path}.")
    return path
