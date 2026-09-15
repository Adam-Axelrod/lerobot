from lerobot.robots.meca500.meca500 import Meca500

from .config_meca500_microscope_basler import Meca500MicroscopeBaslerConfig


class Meca500MicroscopeBasler(Meca500):
    """Same Meca500 robot, microscope rig with the Basler microscope camera.

    Behaviour is fully inherited - Meca500 builds its cameras from `config.cameras`, so the
    camera set flows through unchanged. This subclass exists only so the dynamic device
    loader can resolve `Meca500MicroscopeBaslerConfig` -> `Meca500MicroscopeBasler`.
    """

    config_class = Meca500MicroscopeBaslerConfig
    name = "meca500_microscope_basler"
