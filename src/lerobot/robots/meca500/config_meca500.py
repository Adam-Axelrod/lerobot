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
                index_or_path=1,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

    max_relative_target: float | dict[str, float] | None = None
    default_joint_vel: float = 25.0

    json_path: str = "bota_sensor_config.json"
    sensor_type: str = "Bota_Binary_gen0"
    sensor_port: str = "COM4" # Or /dev/ttyUSB0 if on Mac

    # Hand guidance parameters (Ported from hand_guidance.py)
    gain_tr: int = 10 #10
    gain_rot: int = 50  #50
    f_threshold_high: float = 0.5
    f_threshold_low: float = 0.1
    m_threshold_high: float = 0.05
    m_threshold_low: float = 0.01
    
    alpha: float = 0.1 #  0.1 Simple low pass filter factor


