from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("meca500")
@dataclass
class Meca500Config(RobotConfig):
    # Standard field for connecting to the robot (e.g., /dev/ttyUSB0)
    #port: str

    ip_address: str = "192.168.0.100"
    
    monitor_mode: bool = False

    # Standard camera configuration
    cameras: dict[str, CameraConfig] = field(
        default_factory={
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=480,
                height=640,
            ),
        }
    )

    max_relative_target: float | dict[str, float] | None = None
    default_joint_vel: float = 25.0


