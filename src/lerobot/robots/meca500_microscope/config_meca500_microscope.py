from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config


@RobotConfig.register_subclass("meca500_microscope")
@dataclass
class Meca500MicroscopeConfig(Meca500Config):
    """Meca500 with the microscope/pipette rig: overhead camera (index 0) plus a
    microscope camera (index 2) replacing the wrist camera.

    Identical Meca500 hardware/behaviour — only the camera set differs. Focus and
    exposure are locked (autofocus/autoexposure off) so the visual statistics stay
    consistent between recording and rollout. `focus`/`exposure` are driver-dependent;
    tune per camera/lighting with scripts/meca500/check_camera.py.
    """

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "overhead_cam": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
            "microscope_cam": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                # Native resolution of the USB microscope cam — it snaps any other
                # request to 1280x1024, which the strict width/fps check then rejects.
                width=1280,
                height=1024,
                # MJPG keeps 1.3MP @ 30fps within USB bandwidth alongside the overhead cam.
                fourcc="MJPG",
                # Focus is fixed by the lens ring on this camera — the driver rejects
                # CAP_PROP_FOCUS, so leave focus control untouched (None).
                autofocus=None,
                focus=None,
                autoexposure=False,
                exposure=-6,
            ),
        }
    )
