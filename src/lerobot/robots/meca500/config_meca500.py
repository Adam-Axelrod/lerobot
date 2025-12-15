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
    
    monitor_mode: bool = True

    # Standard camera configuration
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {  # <--- Add lambda: here
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    max_relative_target: float | dict[str, float] | None = None
    default_joint_vel: float = 25.0


