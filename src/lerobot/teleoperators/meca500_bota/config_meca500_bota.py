from dataclasses import dataclass, field

from lerobot.robots.meca500.bota import BOTA_CONFIGS

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("meca500_bota")
@dataclass
class Meca500BotaConfig(TeleoperatorConfig):
    # Port to connect to the arm
    meca_address: str = "192.168.0.100"

    # bota_driver config for the fitted sensor. Defaults to the original setup's MiniONE; Huanyu's
    # setup has a MiniONE Pro - pick per rig with scripts/meca500/rigs.py bota_config_path(rig).
    json_path: str = str(BOTA_CONFIGS["miniONE"])
    sensor_type: str = "Bota_Binary"
    sensor_port: str = "COM5"  # Or /dev/ttyUSB0 if on Mac

    # Hand guidance parameters (Ported from hand_guidance.py)
    gain_tr: int = 10  # 10
    gain_rot: int = 50  # 50
    f_threshold_high: float = 0.5
    f_threshold_low: float = 0.1
    m_threshold_high: float = 0.05
    m_threshold_low: float = 0.01

    alpha: float = 0.1  #  0.1 Simple low pass filter factor

    # Target pose used by go_home() (invoked around every episode by scripts/meca500/record.py).
    home_joints: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 90.0, 0.0])
    home_timeout_s: float = 30.0
