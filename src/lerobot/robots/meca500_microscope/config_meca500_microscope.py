from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import Cv2Backends
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500_microscope.camera_resolver import apply_resolved_indices


@RobotConfig.register_subclass("meca500_microscope")
@dataclass
class Meca500MicroscopeConfig(Meca500Config):
    """Meca500 with the microscope/pipette rig: overhead cam, wrist cam, and a
    USB microscope cam.

    Identical Meca500 hardware/behaviour — only the camera set differs. Focus and
    exposure are locked (autofocus/autoexposure off) so the visual statistics stay
    consistent between recording and rollout. `focus`/`exposure` are driver-dependent;
    tune per camera/lighting with scripts/meca500/check_camera.py.

    Camera indices: the ``index_or_path`` values below are only a *fallback*. USB
    indices are not stable on Windows — unplugging/replugging the microscope (or a
    power-cycle) reshuffles them — so on Windows the indices are re-resolved at
    construction by DirectShow device name (see camera_resolver.py):
      * microscope = "VMS700"  (unique name, always found)
      * overhead + wrist = "UC60 Video"  (same model; told apart only by USB port,
        lower index = overhead, higher = wrist — keep each in its own port)
    The built-in laptop webcam ("Chicony...") is ignored. Off Windows, or if the
    lookup fails, the static indices below are used as-is. Confirm any changes with
    scripts/meca500/check_camera.py.

    NOTE: the microscope cam (VMS700) is a slow 16:9 USB device — it sustains
    ~22fps at 640x360, ~5.5fps at 1280x720, ~2.4fps at 1920x1080 (its native max),
    so it runs at 640x360 here. The two UC60 arm cams sustain 640x480@30 fine.
    """

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "overhead_cam": OpenCVCameraConfig(
                # Fallback index (re-resolved by name on Windows). "UC60 Video", lower index.
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
                # DirectShow, not the CAP_ANY default (which picks MSMF on Windows and
                # fails to open these UVC cams). Matches scripts/meca500/check_camera.py.
                backend=Cv2Backends.DSHOW,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
            "wrist_cam": OpenCVCameraConfig(
                # Fallback index (re-resolved by name on Windows). "UC60 Video", higher index.
                index_or_path=2,
                fps=30,
                width=640,
                height=480,
                # MJPG to fit a third stream alongside overhead + microscope on the bus.
                fourcc="MJPG",
                backend=Cv2Backends.DSHOW,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
            "microscope_cam": OpenCVCameraConfig(
                # Fallback index (re-resolved by name on Windows). "VMS700".
                index_or_path=3,
                fps=30,
                # The VMS700 is a 16:9 sensor; its only MJPG modes are 1920x1080,
                # 1280x720 and 640x360. It's slow, so 640x360 (~22fps) is used for a
                # usable teleop/record rate (720p drops to ~5.5fps, 1080p to ~2.4fps).
                width=640,
                height=360,
                # MJPG keeps the stream within USB bandwidth alongside the other cams.
                fourcc="MJPG",
                backend=Cv2Backends.DSHOW,
                # Focus is fixed by the lens ring on this camera — the driver rejects
                # CAP_PROP_FOCUS, so leave focus control untouched (None).
                autofocus=None,
                focus=None,
                autoexposure=False,
                exposure=-6,
            ),
        }
    )

    def __post_init__(self) -> None:
        # Re-resolve USB camera indices by stable DirectShow name before the base
        # class validates them, so a reshuffle (replug/power-cycle) can't leave a
        # camera pointing at the wrong index. No-op off Windows / on lookup failure.
        apply_resolved_indices(self.cameras)
        super().__post_init__()
