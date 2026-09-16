from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500_microscope.basler_microscope_camera import basler_microscope_camera


@RobotConfig.register_subclass("meca500_basler")
@dataclass
class Meca500BaslerConfig(Meca500Config):
    """Meca500 with the Basler microscope camera and NOTHING else - no arm cameras.

    This is the second bench: the Meca500 is cabled to the laptop's built-in Ethernet
    port through the UGREEN switch, and the only camera present is the Basler down the
    microscope. The two UC60 arm cameras are not on this bench yet.

    Why this is its own rig rather than `meca500_microscope_basler` with two cameras
    removed: the rig name is what ties a recorded dataset and a trained checkpoint to the
    cameras that produced them, and a one-camera observation space is not a three-camera
    one. A policy trained here has no `observation.images.overhead_cam` or
    `.wrist_cam` input at all, so it can neither be evaluated on the three-camera rig nor
    loaded against it. Keeping them separate means the arm cameras can arrive later
    without invalidating anything recorded today.

    When the two UC60 arm cameras do arrive, they belong on `meca500_microscope_basler`
    (which is already written and expects exactly that camera set) - not bolted onto this
    rig, which would redefine what `meca500_basler` means underneath data already recorded.

    The camera itself - serial, binning, output size - is defined in
    basler_microscope_camera.py and shared with the three-camera rig, so the same physical
    camera cannot end up producing different pixels on the two rigs. Exposure and gain live
    in the camera's own stored user set; see that module for the frame-rate constraints
    (binning 2 is required to get above 5.8 fps, and 20 fps needs exposure under ~45 ms).

    There is no USB index resolution here: the Basler is addressed by serial and never
    appears in DirectShow enumeration, so unlike the UC60 rigs there is nothing that a
    replug or power-cycle can reshuffle.
    """

    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {"microscope_cam": basler_microscope_camera()}
    )

    @property
    def camera_settings_key(self) -> str:
        """This rig has no tunable cameras, so it has no section in camera_settings.json.

        The Basler keeps exposure and gain in its own persistent user set, and the base
        class skips a camera backend with no software exposure/focus. Returning the rig
        name (the default) would be harmless but implies a section that should never be
        created - a `meca500_basler` entry in that file could only ever be a mistake.
        """
        return "meca500_basler"
