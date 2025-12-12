# My imports
import mecademicpy.robot as mdr


# Hugging Face imports
import logging
import time
from functools import cached_property
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from .config_meca500 import Meca500Config
from ..utils import ensure_safe_goal_position

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

#remove these
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

logger = logging.getLogger(__name__)

class Meca500(Robot):
    config_class = Meca500Config
    name = "meca500"

    def __init__(self, config: Meca500Config):
        super().__init__(config)
        self.config = config
        self.robot = mdr.Robot()
        self.cameras = make_cameras_from_configs(config.cameras)
        self._connected = False

    @property
    def _motors_ft(self) -> dict[str, type]:
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
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }
    
    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}
    
    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft
    
    @property
    def is_connected(self) -> bool:
        return self._connected and all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting to Meca500 at {self.config.ip_address}...")
        try:
            self.robot.Connect(
                address=self.config.ip_address,
                enable_synchronous_mode=False,
                monitor_mode=False
                )
        except Exception as e:
            raise DeviceNotConnectedError(f"Failed to connect to Meca500 at {self.config.ip_address}: {e}")
        
        logger.info("Activating and Homing Meca500...")
        try:
            self.robot.ActivateAndHome()
            self.robot.WaitHomed()
        except Exception as e:
            raise DeviceNotConnectedError(f"Failed to activate and home Meca500: {e}")
        
        self.configure()

        for cam in self.cameras.values():
            cam.connect()

        self._connected = True
        logger.info(f"{self} connected and homed.")

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
        self.robot.SetBlending(100)

        if self.config.default_joint_vel:
            self.robot.SetJointVel(self.config.default_joint_vel)

        self.robot.SetRealTimeMonitoring('all')

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        # Read arm position
        start = time.perf_counter()

        try:
            joints = self.robot.GetRtTargetJointPos(synchronous_update=True)
        except Exception as e:
            logger.error(f"Failed to read robot state: {e}")
            # Fallback or re-raise depending on strictness required
            raise e

        obs_dict = {
            "joint_1.pos": joints[0],
            "joint_2.pos": joints[1],
            "joint_3.pos": joints[2],
            "joint_4.pos": joints[3],
            "joint_5.pos": joints[4],
            "joint_6.pos": joints[5],
        }

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")
        
        # Read cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
            
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        # Write to hardware
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        
        # Cap goal position when too far away from present position.
        # /!\ Slower fps expected due to reading from the follower.
        if self.config.max_relative_target is not None:
            present_pos = self.robot.GetRtTargetJointPos(synchronous_update=True)
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Send goal position to the arm
        target_joints = [
            action["joint_1.pos"],
            action["joint_2.pos"],
            action["joint_3.pos"],
            action["joint_4.pos"],
            action["joint_5.pos"],
            action["joint_6.pos"],
        ]
        self.robot.MoveJoints(*target_joints)
        return action
    
    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        elif self.robot.IsConnected():
            logger.info("Disconnecting Meca500...")
            # Ideally stop motion before disconnecting
            try:
                self.robot.PauseMotion()
                self.robot.DeactivateRobot()
                self.robot.Disconnect()
            except Exception as e:
                logger.warning(f"Error during disconnect sequence: {e}")

        
        for cam in self.cameras.values():
            cam.disconnect()
        
        self._connected = False
        logger.info(f"{self} disconnected.")
    

