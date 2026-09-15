from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.basler import BaslerCameraConfig
from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig
from lerobot.robots.meca500_microscope.camera_resolver import apply_resolved_indices
from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig

# Serial number of the Basler microscope camera, as printed on its label. Find it with:
#     python scripts/meca500/basler_probe.py --list
# Unlike the USB webcams there is no index to resolve: pylon addresses devices by serial,
# so this value is stable across replugs, ports and reboots. If it is wrong, the camera
# fails to connect with an error listing the serials that *were* found.
MICROSCOPE_CAMERA_SERIAL = "REPLACE_ME"


@RobotConfig.register_subclass("meca500_microscope_basler")
@dataclass
class Meca500MicroscopeBaslerConfig(Meca500MicroscopeConfig):
    """The microscope rig with the Basler microscope camera instead of the VMS700.

    Kept as a *separate* rig rather than replacing `meca500_microscope`, because the rig
    name is what ties a recorded dataset and a trained checkpoint to the cameras that
    produced them. Every microscope dataset and policy up to and including
    `microscope_pipette_2026-09-10` expects `microscope_cam` at 640x360 from the VMS700
    (`observation.images.microscope_cam` has shape [360, 640, 3]); redefining
    `meca500_microscope` under them would make old checkpoints fail to load against the
    rig they were trained on, and old datasets impossible to reproduce.

    So: `meca500_microscope` stays the VMS700 rig, and this is the Basler one. Data
    recorded here needs its own training run - the microscope pixels are not comparable.

    The arm cameras are unchanged and still resolved by USB device identity (see
    camera_resolver.py). The Basler is exempt from that: it is a USB3-Vision/GigE device
    driven through pypylon, so it never appears in DirectShow enumeration and is addressed
    by serial number instead of by index.

    NOTE: for the Basler, ``width``/``height`` are the **output** frame size, not a sensor
    ROI. Writing Width/Height on a Basler crops the sensor rather than scaling it, which on
    a microscope would throw away most of the field of view, so BaslerCamera grabs the full
    sensor and downscales to the size below. Pick a width/height matching the sensor's own
    aspect ratio, or the image is anisotropically squashed - check what the sensor reports:
        python scripts/meca500/basler_probe.py --serial <serial>
    Exposure/gain are not set here; the camera keeps its stored user set (tune it in the
    pylon Viewer and "Save User Set"), which is why it has no entry in camera_settings.json.
    """

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "overhead_cam": OpenCVCameraConfig(
                # Fallback index only (re-resolved by USB device id on Windows). "UC60 Video".
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
                # DirectShow, not the CAP_ANY default (which picks MSMF on Windows and
                # fails to open these UVC cams). Matches scripts/meca500/camera_tool.py.
                backend=Cv2Backends.DSHOW,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
            "wrist_cam": OpenCVCameraConfig(
                # Fallback index only (re-resolved by USB device id on Windows). "UC60 Video".
                index_or_path=2,
                fps=30,
                width=640,
                height=480,
                # MJPG to fit both webcam streams on the shared bus.
                fourcc="MJPG",
                backend=Cv2Backends.DSHOW,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
            "microscope_cam": BaslerCameraConfig(
                serial_number=MICROSCOPE_CAMERA_SERIAL,
                fps=30,
                # Output size (full sensor, downscaled - see the class docstring). 4:3 here;
                # revisit once the sensor's native aspect ratio is confirmed with
                # `basler_probe.py --serial <serial>`.
                width=640,
                height=480,
            ),
        }
    )

    @property
    def camera_settings_key(self) -> str:
        """Share the VMS700 rig's tuned exposure/focus.

        The two arm cameras are the same physical units with the same settings, so they
        read the same section rather than a duplicate that would drift. The Basler has no
        entry there at all - its exposure and gain live in the camera's own stored user set.
        """
        return "meca500_microscope"

    def __post_init__(self) -> None:
        # Re-resolve the USB camera indices by stable device identity before the base class
        # validates them. Only the index-addressed roles are looked up, so the Basler does
        # not trigger a "microscope not found in DirectShow" warning that would always fire.
        apply_resolved_indices(self.cameras)
        # Skip Meca500MicroscopeConfig.__post_init__, which resolves indices for the VMS700
        # rig, and go straight to the Meca500 base (camera settings + validation).
        super(Meca500MicroscopeConfig, self).__post_init__()
