from lerobot.robots.meca500.meca500 import Meca500

from .config_meca500_basler import Meca500BaslerConfig


class Meca500Basler(Meca500):
    """Same Meca500 robot, Basler microscope camera only (no arm cameras).

    Behaviour is fully inherited - Meca500 builds its cameras from `config.cameras`, so the
    single-camera set flows through unchanged. This subclass exists only so the dynamic
    device loader can resolve `Meca500BaslerConfig` -> `Meca500Basler`.
    """

    config_class = Meca500BaslerConfig
    name = "meca500_basler"
