from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("meca500")
@dataclass
class Meca500Config(RobotConfig):
    # Standard field for connecting to the robot (e.g., /dev/ttyUSB0)
    port: str

    ip_address: str = "192.168.0.100"

    disable_torque_on_disconnect: bool = True

    # `max_relative_target` limits the magnitude of the relative positional target vector for safety purposes.
    # Set this to a positive scalar to have the same value for all motors, or a dictionary that maps motor
    # names to the max_relative_target value for that motor.
    max_relative_target: float | dict[str, float] | None = None

    # Standard camera configuration
    cameras: dict[str, CameraConfig] = field(
        default_factory={
            "cam_1": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                width=480,
                height=640,
            ),
        }
    )

    # Set to `True` for backward compatibility with previous policies/dataset
    use_degrees: bool = False


