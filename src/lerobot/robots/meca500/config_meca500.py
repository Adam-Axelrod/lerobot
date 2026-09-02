from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import RobotConfig
from lerobot.robots.meca500.camera_settings import apply_camera_settings


@RobotConfig.register_subclass("meca500")
@dataclass
class Meca500Config(RobotConfig):
    ip_address: str = "192.168.0.100"

    monitor_mode: bool = True

    # Standard camera configuration.
    # Focus and exposure are locked (autofocus/autoexposure off) so the visual
    # statistics stay consistent between recording and rollout.
    #
    # The `focus`/`exposure` values below are only the fallback: the tuned values
    # live in camera_settings.json and are applied in __post_init__, so tuning once
    # with scripts/meca500/camera_tool.py applies to recording, teleop and inference
    # alike. Structural settings (resolution, fps, fourcc, backend) stay here.
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
            "wrist_cam": OpenCVCameraConfig(
                index_or_path=2,
                fps=30,
                width=640,
                height=480,
                autofocus=False,
                focus=100,
                autoexposure=False,
                exposure=-6,
            ),
        }
    )

    max_relative_target: float | dict[str, float] | None = None

    # Tighter per-step joint-delta clamp applied at deployment when the policy's
    # predicted `precision.state` channel is active (> 0.5). Lets a trained policy
    # "scale down its own movements" under the microscope. None disables the gate
    # (the normal max_relative_target still applies). Tune after the first checkpoint.
    precision_max_relative_target: float | dict[str, float] | None = None

    default_joint_vel: float = 25.0

    def __post_init__(self) -> None:
        # Apply the tuned exposure/focus for this rig before the base class validates
        # the camera set. Keyed on `self.type` (the registered choice name), so the
        # microscope subclass automatically picks up its own section instead of the
        # base rig's - no per-subclass wiring, and no chance of the two crossing over.
        apply_camera_settings(self.type, self.cameras)
        super().__post_init__()
