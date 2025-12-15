import logging
import time
import threading
import numpy as np
import json
from typing import Optional, Dict, Any

import bota_driver
import mecademicpy.robot as mdr


from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_meca500_bota import meca500BotaConfig

logger = logging.getLogger(__name__)


class meca500_bota(Teleoperator):
    config_class = meca500BotaConfig
    name = "meca500_bota"

    def __init__(self, config: meca500BotaConfig):
        super().__init__(config)
        self.config = config
        self.sensor: bota_driver.BotaDriver | None = None
        self.robot = mdr.Robot()

        self._running = False
        self._thread = None
        self.connected = False

        self.wrench_filter = np.zeros(6)


    @property
    def action_features(self) -> dict[str, type]:
        # Define what data your robot provides (key: type/shape)
        # Example: {"joint_1.pos": float, "camera_front": (480, 640, 3)}
        return {
            "joint_1.pos": float,
            "joint_2.pos": float,
            "joint_3.pos": float,
            "joint_4.pos": float,
            "joint_5.pos": float,
            "joint_6.pos": float,
        }

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.connected
    
    def read_json(self, path: str) -> Dict[str, Any]:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        
        try:
            logger.info(f"Connecting to Bota Sensor")
            config_content = self.read_json(self.config.json_path)
            self.sensor = bota_driver.BotaDriver(self.config.json_path)
            # Transition driver from UNCONFIGURED to INACTIVE state
            if not self.sensor.configure():
                raise RuntimeError("Failed to configure driver")
            # Tare the sensor
            if not self.sensor.tare():
                raise RuntimeError("Failed to tare sensor")
            # Transition driver from INACTIVE to ACTIVE state
            if not self.sensor.activate():
                raise RuntimeError("Failed to activate driver")
            
        except Exception as e:
            raise DeviceNotConnectedError(f"Failed to connect to Bota Sensor at {self.config.sensor_port}: {e}")

        try:
            logger.info(f"Connecting to Meca500 (Control) at {self.config.meca_address}...")
            self.robot.Connect(address=self.config.meca_address)
            self.robot.ActivateAndHome()
            self.robot.WaitHomed()
        except Exception as e:
            raise DeviceNotConnectedError(f"Failed to connect to Meca500 at {self.config.meca_address}: {e}")

        self.connected = True
        self._running = True
        self._thread = threading.Thread(target=self._guidance_loop, daemon=True)
        self._thread.start()
        logger.info("Hand Guidance Started.")

        logger.info(f"{self} connected.")

    def _guidance_loop(self):
        
        while self._running:
            # Read sensor
            frame_data = self.sensor.read_frame()
            if frame_data: # and frame_data.status == bota_driver.Status.VALID:
                wrench = np.array([
                    frame_data.forces[0], frame_data.forces[1], frame_data.forces[2],
                    frame_data.torques[0], frame_data.torques[1], frame_data.torques[2]
                ])
                
                # Filter
                self.wrench_filter = (1 - self.config.alpha) * self.wrench_filter + self.config.alpha * wrench
                
                # Logic from your script (simplified for brevity)
                normF = np.linalg.norm(self.wrench_filter[:3])
                normM = np.linalg.norm(self.wrench_filter[3:])
                
                twist = np.zeros(6)
                
                # Translation
                if normF > self.config.f_threshold_high:
                     twist[:3] = self.config.gain_tr * self.wrench_filter[:3]
                
                # Rotation
                if normM > self.config.m_threshold_high:
                     twist[3:] = self.config.gain_rot * self.wrench_filter[3:]

                # Send Velocity to Robot
                # MoveLinVelTrf expects: x, y, z, wx, wy, wz
                self.robot.MoveLinVelTrf(
                    -twist[0], -twist[1], twist[2], 
                    -twist[3], -twist[4], twist[5]
                )
            
            time.sleep(0.002) # ~500Hz loop

    @property
    def is_calibrated(self) -> bool:
        if not self.is_connected:
            return False
        return True  # If its homed, its calibrated

    def calibrate(self) -> None:
        # For Meca500, calibration is "Homing"
        if self.is_connected:
            logger.info("Homing robot...")
            self.robot.Home()
            self.robot.WaitHomed(timeout=60)

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        joints = self.robot.GetRtTargetJointPos(synchronous_update=True).data
        return {
            "joint_1.pos": joints[0],
            "joint_2.pos": joints[1],
            "joint_3.pos": joints[2],
            "joint_4.pos": joints[3],
            "joint_5.pos": joints[4],
            "joint_6.pos": joints[5],
        }

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        
        if self.robot.IsConnected():
            self.robot.Disconnect()
            self.robot.WaitDisconnected()
            logger.info("Meca500 disconnected.")

        if self.sensor:
            self.sensor.deactivate()
            self.sensor.shutdown()
