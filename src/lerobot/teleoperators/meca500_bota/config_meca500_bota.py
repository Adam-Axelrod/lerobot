from dataclasses import dataclass

from ..config import TeleoperatorConfig
import numpy as np


@TeleoperatorConfig.register_subclass("meca500_bota")
@dataclass
class meca500BotaConfig(TeleoperatorConfig):
    # Port to connect to the arm
    meca_address: str = "192.168.0.100"

    json_path: str = "bota_sensor_config.json"
    sensor_type = "Bota_Binary_gen0"
    sensor_port: str = "COM4" # Or /dev/ttyUSB0 if on Mac

    # Hand guidance parameters (Ported from hand_guidance.py)
    gain_tr = 10 #10
    gain_rot = 50  #50
    f_threshold_high = 0.5
    f_threshold_low = 0.1
    m_threshold_high = 0.05
    m_threshold_low = 0.01
    
    wrench_filter = np.zeros(6)
    alpha = 0.1 #  0.1 Simple low pass filter factor
    
    