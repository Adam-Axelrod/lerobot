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
                monitor_mode=self.config.monitor_mode
                )
        except Exception as e:
            raise DeviceNotConnectedError(f"Failed to connect to Meca500 at {self.config.ip_address}: {e}")
        
        if not self.config.monitor_mode:
            logger.info("Activating and Homing Meca500...")
            try:
                self.robot.ActivateAndHome()
                self.robot.WaitHomed()
                self.configure()
            except Exception as e:
                raise DeviceNotConnectedError(f"Failed to activate and home Meca500: {e}")

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
            joints = self.robot.GetRtTargetJointPos(synchronous_update=True).data
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
    
    def _get_motion_queue_count(self) -> int:
        """Helper to query the robot for the number of pending motion commands."""
        # 2080 is the code for MX_ST_GET_CMD_PENDING_COUNT
        try:
            response = self.robot.SendCustomCommand("GetCmdPendingCount", expected_responses=[2080], timeout=0.5)
            # The data field of the message contains the count as a string/int
            return int(response.data)
        except Exception as e:
            logger.warning(f"Failed to get queue count: {e}")
            return 100 # Return high number to prevent spamming if check fails

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        if self.config.monitor_mode:
            # In monitor mode, we do not send any commands
            return action
        
        # 1. Manage the Motion Queue
        # We only send a new command if the robot is running low on commands.
        # This prevents filling the buffer (and creating massive latency).
        # A buffer of 1 or 2 is usually sufficient for smooth motion with blending.
        queue_count = self._get_motion_queue_count()
        if queue_count > 1:
            # Skip sending this action frame to let the robot catch up
            return action
        

        # 2. Parse Actions
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        
        # 3. Safety Cap (Optional)
        if self.config.max_relative_target is not None:
            present_pos_list = self.robot.GetRtTargetJointPos(synchronous_update=True).data
            present_pos = {f"joint_{i+1}": p for i, p in enumerate(present_pos_list)}
            
            goal_present_pos = {key: (goal_pos[key], present_pos[key]) for key in goal_pos}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # 4. Send Command
        # Note: Meca500 MoveJoints takes args, not a list
        self.robot.MoveJoints(
            goal_pos["joint_1"],
            goal_pos["joint_2"],
            goal_pos["joint_3"],
            goal_pos["joint_4"],
            goal_pos["joint_5"],
            goal_pos["joint_6"]
        )
        return action
    
    def disconnect(self) -> None:
        if not self.is_connected:
            return
            
        logger.info("Disconnecting Meca500...")
        try:
            if self.robot.IsConnected():
                self.robot.PauseMotion()
                self.robot.ClearMotion()
                self.robot.DeactivateRobot()
                self.robot.Disconnect()
        except Exception as e:
            logger.warning(f"Error during disconnect sequence: {e}")

        for cam in self.cameras.values():
            cam.disconnect()
        
        self._connected = False
        logger.info(f"{self} disconnected.")
    

