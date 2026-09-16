# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass

from ..configs import CameraConfig, ColorMode, Cv2Rotation

__all__ = ["BaslerCameraConfig", "ColorMode", "Cv2Rotation"]


@CameraConfig.register_subclass("basler")
@dataclass
class BaslerCameraConfig(CameraConfig):
    """Configuration for a Basler (pylon/GenICam) camera, driven through pypylon.

    Example:
    ```python
    BaslerCameraConfig(serial_number="40012345", fps=30, width=640, height=480)
    ```

    Attributes:
        serial_number: The camera's serial number, as printed on its label and reported by
            `python scripts/meca500/basler_probe.py --list`. Basler devices are always
            addressed by serial, never by enumeration index: the pylon docs are explicit
            that index order changes with the number of connected cameras.
        fps: Requested frames per second. Applied via `AcquisitionFrameRate`, clipped to
            what the camera can actually sustain at the current exposure. None leaves the
            camera's own rate untouched.
        width: Width of the frames this camera *outputs*, in pixels. See the note below.
        height: Height of the frames this camera *outputs*, in pixels.
        color_mode: Color mode for image output (RGB or BGR). Defaults to RGB.
        rotation: Image rotation setting (0deg, 90deg, 180deg, or 270deg). Defaults to none.
        binning: Sensor-side binning factor (1 = off, 2 = 2x2, ...). Combines neighbouring
            pixels on the camera, which cuts bus bandwidth and improves low-light SNR at the
            cost of resolution. Leave at 1 unless the link cannot sustain the full sensor.
        reverse_x: Mirror the image left-right, on the sensor (`ReverseX`).
        reverse_y: Mirror the image top-bottom, on the sensor (`ReverseY`).
        warmup_s: Time spent reading frames before returning from connect (in seconds).

    Note:
        `width`/`height` are the **output** frame size, not a sensor ROI. On a Basler,
        writing `Width`/`Height` crops the sensor rather than scaling it, which on a
        microscope would throw away most of the field of view. This backend therefore grabs
        the *full* sensor (after `binning`) and downscales to `width` x `height`, matching
        the semantics of `OpenCVCameraConfig` so the rest of the pipeline is unaffected.
        Pick an aspect ratio matching the sensor's, or the image is anisotropically squashed.

        Exposure and gain are deliberately not configurable here. The camera keeps whatever
        is in its persistent user set - tune it in the pylon Viewer and `Save User Set`.

        `reverse_x`/`reverse_y` ARE configured here rather than left to the user set, unlike
        exposure and gain, because they are structural: they change what a pixel coordinate
        means, so a dataset is only interpretable alongside the orientation that recorded it.
        Keeping them on the dataclass puts them in version control next to the rig. They are
        applied on the sensor, so they cost nothing and happen before the downscale.

        `reverse_x` + `reverse_y` together is a 180-degree rotation, so prefer them over
        `rotation=ROTATE_180` (same result, no CPU). `rotation` remains the only way to get
        90 or 270 degrees, which swap the axes rather than mirroring them.
    """

    serial_number: str
    color_mode: ColorMode = ColorMode.RGB
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    binning: int = 1
    reverse_x: bool = False
    reverse_y: bool = False
    warmup_s: int = 1

    def __post_init__(self) -> None:
        self.color_mode = ColorMode(self.color_mode)
        self.rotation = Cv2Rotation(self.rotation)

        # The serial is a string on the wire; accept an int for convenience but normalise,
        # otherwise the DeviceInfo match silently fails against pylon's string comparison.
        self.serial_number = str(self.serial_number)
        if not self.serial_number:
            raise ValueError("`serial_number` is required; run scripts/meca500/basler_probe.py --list.")

        if not isinstance(self.binning, int) or self.binning < 1:
            raise ValueError(f"`binning` must be an integer >= 1, but {self.binning} is provided.")

        # Guard against a truthy non-bool (e.g. reverse_x=1) silently working here but not
        # round-tripping through a config file as the same type.
        for name in ("reverse_x", "reverse_y"):
            if not isinstance(getattr(self, name), bool):
                raise ValueError(f"`{name}` must be a bool, but {getattr(self, name)!r} is provided.")

        if (self.width is None) != (self.height is None):
            raise ValueError(
                "`width` and `height` must be set together (or both left None to use the full "
                f"sensor size), but width={self.width} and height={self.height} were provided."
            )
