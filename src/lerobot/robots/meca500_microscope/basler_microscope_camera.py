"""The Basler microscope camera, defined once for every rig that mounts it.

Two rigs use this camera - ``meca500_basler`` (microscope only) and
``meca500_microscope_basler`` (microscope + the two arm cams) - and they must agree on
its serial, binning and output size, or the same physical camera would produce
differently-shaped pixels on the two rigs and their datasets would silently stop being
comparable. That is exactly the drift ``camera_settings.py`` exists to prevent, so the
definition lives here and both configs call :func:`basler_microscope_camera`.

Measured on the unit below (acA5472-17ucMED, 20MP, 3:2 sensor), grabbing full-sensor
frames over USB3:

    binning 1  5496x3660   5.8 fps   <- bandwidth bound; exposure makes NO difference
    binning 2  2748x1830  10.8 fps   <- exposure bound, at the auto-exposed 90.9 ms
    binning 2  2748x1830  23.0 fps   <- same, with exposure forced to 20 ms

Hence ``BINNING = 2``: at binning 1 the link saturates at 5.8 fps and no amount of light
will lift it, which is below every recording rate this rig is used at. Binning combines
neighbouring pixels *on the sensor*, so unlike a Width/Height ROI it keeps the entire
field of view - the whole point on a microscope.

The remaining limit is exposure: the camera cannot exceed ``1 / exposure_time``, so 20 fps
needs exposure under ~45 ms. Exposure and gain are deliberately NOT set here - they live in
the camera's own persistent user set (tune in the pylon Viewer, then "Save User Set"), the
same split the rest of this package uses. If recording runs slow, check exposure first:
    python scripts/meca500/basler_probe.py --serial 40585204
There is ~27 dB of gain headroom, so more light or more gain both buy frame rate.
"""

from __future__ import annotations

from lerobot.cameras.basler import BaslerCameraConfig
from lerobot.cameras.configs import Cv2Rotation

# Serial as printed on the camera label; confirm with `basler_probe.py --list`. pylon
# addresses devices by serial, so this is stable across replugs, ports and reboots.
MICROSCOPE_CAMERA_SERIAL = "40585204"

# See the module docstring: 1 is bandwidth-capped at 5.8 fps, 2 reaches 23 fps.
BINNING = 2

# Output size, AFTER the rotation below. The sensor is 3:2 (5484:3660 = 1.498) and the
# quarter turn swaps the axes, so the corrected frame is 2:3 portrait: 427 wide by 640 tall.
# BaslerCamera resizes to the pre-rotation pair for you, so the full field of view survives -
# these are output dims, not a crop.
WIDTH = 427
HEIGHT = 640

# ---------------------------------------------------------------- image orientation
#
# This is an INVERTED microscope AND the camera sits a quarter turn out on the port, so the
# raw image is rotated relative to the sample. Teleoperating against that is the classic
# "push right, it goes up" problem, and it costs accuracy on every fine approach.
#
# A mirror cannot fix a rotation: ReverseX/ReverseY swap a direction but never swap the
# AXES, so "moving the sample right moves the image up" can only be corrected by a rotation.
# Hence the rotation below rather than more reversing.
#
# CAREFUL with the reverses: they happen on the SENSOR, before the rotation. With a quarter
# turn in play the axes have already swapped by the time you see the frame, so here
#
#     REVERSE_X mirrors the displayed image VERTICALLY
#     REVERSE_Y mirrors the displayed image HORIZONTALLY
#
# which is why the horizontal mirror this rig needs is REVERSE_Y, not the REVERSE_X you would
# reach for. (At NO_ROTATION or ROTATE_180 they mean what their names say.) Reading them as
# display-space flips is the easy mistake, and it costs a full record-and-look cycle.
#
# Only one reverse is ever needed: two reverses are a 180-degree rotation, already reachable
# through ROTATION, and two ways to spell one orientation is how these drift. All eight
# orientations are covered by the four rotations times one mirror.
#
# VERIFY before recording, because only the physical rig can settle it:
#     python scripts/meca500/camera_tool.py --rig meca500_basler
# Move the sample right, then away from you, and watch the preview.
#   * image follows the sample on both axes -> correct, leave these alone
#   * image is a quarter turn out           -> step ROTATION a quarter turn
#   * axes right but the HORIZONTAL opposes -> toggle REVERSE_Y  (yes, Y)
#   * axes right but the VERTICAL opposes   -> toggle REVERSE_X  (yes, X)
#
# Settle it BEFORE recording. These change what a pixel coordinate means, so a dataset is
# only comparable with others recorded at the same orientation - changing this after the
# fact silently invalidates every episode already on disk and any policy trained on them.
ROTATION = Cv2Rotation.ROTATE_270
REVERSE_X = False
REVERSE_Y = True  # horizontal mirror in display space - see the note above


def basler_microscope_camera(fps: int = 30) -> BaslerCameraConfig:
    """The shared Basler microscope camera config.

    Args:
        fps: Requested rate. The camera clips this to what the current exposure allows,
            and warns if it cannot sustain it, so asking for more than the link or the
            exposure permits is safe - it just logs what it actually achieved.
    """
    return BaslerCameraConfig(
        serial_number=MICROSCOPE_CAMERA_SERIAL,
        fps=fps,
        width=WIDTH,
        height=HEIGHT,
        binning=BINNING,
        rotation=ROTATION,
        reverse_x=REVERSE_X,
        reverse_y=REVERSE_Y,
        # The first frame after StartGrabbing waits for one full exposure. With the auto-exposed
        # user set and a dim field (lamp off or low) that took over the default 1 s and connect()
        # timed out mid-record_auto (2026-10-27); normally connect takes 1.5 s in total.
        warmup_s=3,
    )
