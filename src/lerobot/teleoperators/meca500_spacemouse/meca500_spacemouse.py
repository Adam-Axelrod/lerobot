import logging
import threading
import time

import mecademicpy.robot as mdr
import numpy as np

from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..teleoperator import Teleoperator
from .config_meca500_spacemouse import meca500SpacemouseConfig

logger = logging.getLogger(__name__)


class meca500Spacemouse(Teleoperator):
    config_class = meca500SpacemouseConfig
    name = "meca500_spacemouse"

    def __init__(self, config: meca500SpacemouseConfig):
        super().__init__(config)
        self.config = config
        self.robot = mdr.Robot()

        self._running = False
        self._thread: threading.Thread | None = None
        self._connected = False
        # Set while go_home() is queueing a MoveJoints command so the velocity
        # loop parks itself and doesn't fight the homing motion.
        self._homing = threading.Event()

        self._twist_filter = np.zeros(6)
        self._signs = np.array(self.config.axis_signs, dtype=float)
        self._sm_device = None  # SpaceMouseDevice handle, opened in connect()

    @property
    def action_features(self) -> dict[str, type]:
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
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        try:
            import pyspacemouse  # lazy: only required when this teleop is actually used
        except ImportError as e:
            raise DeviceNotConnectedError(
                "pyspacemouse is not installed. Install it with `pip install pyspacemouse` "
                "and ensure hidapi.dll is on the loader path."
            ) from e

        try:
            logger.info("Opening SpaceMouse...")
            self._sm_device = pyspacemouse.open()
            if self._sm_device is None:
                raise RuntimeError("pyspacemouse.open() returned None (device not found?)")
        except Exception as e:
            self._sm_device = None
            raise DeviceNotConnectedError(f"Failed to open SpaceMouse: {e}")

        try:
            logger.info(f"Connecting to Meca500 (Control) at {self.config.meca_address}...")
            self.robot.Connect(address=self.config.meca_address)
            self.robot.ActivateAndHome()
            self.robot.WaitHomed()
        except Exception as e:
            try:
                self._sm_device.close()
            except Exception:
                pass
            self._sm_device = None
            raise DeviceNotConnectedError(f"Failed to connect to Meca500 at {self.config.meca_address}: {e}")

        self._connected = True
        self._running = True
        self._thread = threading.Thread(target=self._guidance_loop, daemon=True)
        self._thread.start()
        logger.info("SpaceMouse teleop started.")
        logger.info(f"{self} connected.")

    def _read_twist(self) -> np.ndarray:
        # SpaceMouseDevice.read() returns a SpaceMouseState with fields
        # t, x, y, z, roll, pitch, yaw, buttons. Axis values are in [-1, 1].
        # SpaceMouse roll/pitch are swapped relative to the Meca's wx/wy,
        # so feed pitch -> wx and roll -> wy. Re-verify empirically after the
        # Trf -> Wrf switch; axis_signs in the config may also need re-tuning.
        state = self._sm_device.read()
        if state is None:
            return np.zeros(6)
        return np.array([state.x, state.y, state.z, state.pitch, state.roll, state.yaw], dtype=float)

    def _guidance_loop(self) -> None:
        dz_tr = self.config.deadzone_tr
        dz_rot = self.config.deadzone_rot
        gain_tr = self.config.gain_tr
        gain_rot = self.config.gain_rot
        alpha = self.config.alpha

        while self._running:
            if self._homing.is_set():
                time.sleep(0.01)
                continue

            raw = self._read_twist() * self._signs
            self._twist_filter = (1 - alpha) * self._twist_filter + alpha * raw

            twist = self._twist_filter.copy()
            # Per-axis deadzone (independent, not magnitude-based — SpaceMouse
            # users routinely push one axis while resting near zero on others).
            twist[:3] = np.where(np.abs(twist[:3]) < dz_tr, 0.0, twist[:3])
            twist[3:] = np.where(np.abs(twist[3:]) < dz_rot, 0.0, twist[3:])

            vx, vy, vz = gain_tr * twist[:3]
            wx, wy, wz = gain_rot * twist[3:]

            self.robot.MoveLinVelWrf(vx, vy, vz, wx, wy, wz)

            time.sleep(self.config.loop_dt)

    @property
    def is_calibrated(self) -> bool:
        return self.is_connected

    def calibrate(self) -> None:
        if self.is_connected:
            logger.info("Homing robot...")
            self.robot.Home()
            self.robot.WaitHomed(timeout=60)

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        joints = self.robot.GetJoints()
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

    def go_home(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        if len(self.config.home_joints) != 6:
            raise ValueError(
                f"home_joints must have 6 values, got {len(self.config.home_joints)}: {self.config.home_joints}"
            )

        home = [float(j) for j in self.config.home_joints]
        logger.info(f"Auto-homing Meca500 to {home}")
        self._homing.set()
        try:
            time.sleep(0.02)
            self.robot.MoveLinVelWrf(0, 0, 0, 0, 0, 0)
            self.robot.MoveJoints(*home)
            self.robot.WaitIdle(timeout=self.config.home_timeout_s)
            logger.info("Home reached.")
        except Exception as e:
            logger.warning(f"Auto-home failed: {e}")
        finally:
            # Drop any pent-up SpaceMouse input so the arm doesn't lurch when guidance resumes.
            self._twist_filter = np.zeros(6)
            self._homing.clear()

    def disconnect(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.robot.IsConnected():
            try:
                self.robot.PauseMotion()
                self.robot.WaitMotionPaused(timeout=5)
                self.robot.ClearMotion()
                self.robot.WaitMotionCleared(timeout=5)
                self.robot.DeactivateRobot()
                self.robot.WaitDeactivated(timeout=10)
            except Exception as e:
                logger.warning(f"Meca500 graceful stop failed, disconnecting anyway: {e}")
            try:
                self.robot.Disconnect()
                self.robot.WaitDisconnected(timeout=5)
                logger.info("Meca500 disconnected.")
            except Exception as e:
                logger.warning(f"Meca500 Disconnect failed: {e}")

        if self._sm_device is not None:
            try:
                self._sm_device.close()
            except Exception as e:
                logger.warning(f"SpaceMouse close failed: {e}")
            self._sm_device = None

        self._connected = False
