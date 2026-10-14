import logging
import time
from functools import cached_property
from typing import Any

import mecademicpy.robot as mdr
import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..utils import ensure_safe_goal_position
from .config_meca500 import Meca500Config

logger = logging.getLogger(__name__)

# How stale a buffered camera frame may be before get_observation() stops trusting it
# and blocks for a fresh one. Loose enough never to trip in normal operation (the rig's
# cameras run at 30fps, so ~33ms between frames) but tight enough that a stalled capture
# thread surfaces as a warning instead of the policy quietly acting on an old image.
CAMERA_MAX_FRAME_AGE_MS = 200


class Meca500(Robot):
    config_class = Meca500Config
    name = "meca500"

    def __init__(self, config: Meca500Config):
        super().__init__(config)
        self.config = config
        self.robot = mdr.Robot()
        self.cameras = make_cameras_from_configs(config.cameras)
        self._connected = False
        # Last precision-gate state, so send_action can log the ON/OFF transitions
        # instead of a line every control step. None = nothing logged yet.
        self._precision_logged: bool | None = None
        # Whether the "this action is absolute, not a delta" note has been logged.
        self._absolute_action_logged: bool = False

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
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        # 6 joint targets plus the latched precision-mode flag (see the SpaceMouse
        # teleop). `precision.state` is not a `.pos` key, so send_action ignores it
        # as a joint target while it is still recorded and predicted by the policy.
        return {**self._motors_ft, "precision.state": float}

    @property
    def is_connected(self) -> bool:
        # Robot-level connectivity should succeed even if one or more optional cameras
        # fail to initialize (e.g., missing wrist camera on this setup).
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting to Meca500 at {self.config.ip_address}...")
        try:
            self.robot.Connect(
                address=self.config.ip_address,
                enable_synchronous_mode=False,
                monitor_mode=self.config.monitor_mode,
            )
            if not self.config.monitor_mode:
                # A Meca500 latches its error state across connections: once a command
                # has been rejected (a joint-limit violation, say), every later command
                # returns MX_ST_ALREADY_ERR until the error is explicitly cleared, and
                # ActivateAndHome() is no exception. Clearing it here is what lets
                # reset.py recover an arm that a previous run left faulted, instead of
                # needing the web interface.
                if self.robot.GetStatusRobot().error_status:
                    logger.warning("Robot is in an error state; resetting it before activating.")
                    self.robot.ResetError()
                    self.robot.WaitErrorReset(timeout=10)
                    self.robot.ResumeMotion()

                logger.info("Homing robot...")
                self.robot.ActivateAndHome()
                self.robot.WaitHomed(timeout=60)

                # A previous controlling program that went away mid-motion (Ctrl-C, crash,
                # cable) leaves the arm PAUSED by the "control connection dropped" safety
                # stop. That is not an error, so the reset above misses it, and every
                # MoveJoints would queue behind the pause: the arm never moves. Clear any
                # stale queued motion and resume - the SpaceMouse teleop does the same.
                if self.robot.GetStatusRobot().pause_motion_status:
                    logger.warning("Robot motion is paused (e.g. a dropped control connection); resuming.")
                    self.robot.ClearMotion()
                    self.robot.ResumeMotion()
        except Exception as e:
            # The exception that surfaces here is a generic DisconnectError; the real
            # cause ("Another user is already controlling the robot") lives further down
            # the __cause__/__context__ chain, so walk it to find the actual message.
            chain = []
            cause = e
            seen = set()
            while cause is not None and id(cause) not in seen:
                seen.add(id(cause))
                chain.append(str(cause))
                cause = cause.__cause__ or cause.__context__
            if any("Another user is already controlling the robot" in msg for msg in chain):
                raise DeviceNotConnectedError(
                    f"Meca500 at {self.config.ip_address} is already controlled by another session. "
                    f"Close the other connection (e.g. the web interface or a running script) and try again."
                ) from None
            raise DeviceNotConnectedError(
                f"Failed to connect to Meca500 at {self.config.ip_address}: {e}"
            ) from e

        failed_cams: list[str] = []
        for cam_name, cam in self.cameras.items():
            try:
                cam.connect()
                logger.info(f"Camera '{cam_name}' connected successfully.")
            except ConnectionError as e:
                logger.warning(
                    f"Failed to connect camera '{cam_name}' (configured as {cam}): {e}. "
                    "This camera will be skipped. Run 'lerobot-find-cameras opencv' to inspect available cameras."
                )
                failed_cams.append(cam_name)

        self._connected = True

        if failed_cams:
            logger.warning(f"Meca500 connected with {len(failed_cams)} unavailable camera(s): {failed_cams}")

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
            self.robot.ActivateAndHome()
            self.robot.WaitHomed(timeout=60)

    def configure(self) -> None:
        self.robot.SetBlending(100)

        if self.config.default_joint_vel:
            self.robot.SetJointVel(self.config.default_joint_vel)

        self.robot.SetRealTimeMonitoring("all")

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        start = time.perf_counter()

        try:
            joints = self.robot.GetJoints()
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

            if cam.is_connected:
                # read_latest(), not async_read(): async_read blocks until the NEXT
                # frame arrives, so at 30fps cameras it costs up to ~33ms of every
                # control step (measured: 32ms median, 48ms p95). Against a 50ms
                # budget at 20fps that alone dropped the loop to ~19Hz once
                # every-step inference (temporal ensembling) took its 15ms.
                #
                # This also *reduces* end-to-end latency rather than trading it away:
                # blocking does not give a fresher image at the moment the action
                # lands, it just delays the action while the world keeps moving.
                # Acting now on a ~17ms-old frame beats waiting 33ms for a 0ms-old one.
                try:
                    obs_dict[cam_key] = cam.read_latest(max_age_ms=CAMERA_MAX_FRAME_AGE_MS)
                except (RuntimeError, TimeoutError) as e:
                    # RuntimeError: no frame buffered yet (first tick after connect).
                    # TimeoutError: the frame is older than the threshold, i.e. the
                    # capture thread has stalled. Either way fall back to a blocking
                    # read so we get a real frame instead of silently reusing a stale
                    # one -- a hung camera must cost time, not go unnoticed.
                    logger.warning(f"{self} {cam_key} read_latest fell back to async_read: {e}")
                    obs_dict[cam_key] = cam.async_read()
            else:
                logger.warning(
                    f"Camera '{cam_key}' is not connected. Returning a blank frame for observation feature consistency."
                )
                blank_height = int(cam.height or getattr(cam, "capture_height", None) or 1)
                blank_width = int(cam.width or getattr(cam, "capture_width", None) or 1)
                obs_dict[cam_key] = np.zeros((blank_height, blank_width, 3), dtype=np.uint8)

            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.config.monitor_mode:
            return action

        # Parse actions
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # When the policy predicts precision mode, clamp per-step motion harder so the
        # arm scales down its own movements under the microscope. Falls back to the
        # normal clamp when the gate is inactive or unconfigured.
        precision_active = float(action.get("precision.state", 0.0)) > 0.5
        relative_target = self.config.max_relative_target
        if precision_active and self.config.precision_max_relative_target is not None:
            relative_target = self.config.precision_max_relative_target

        # Report the gate only when it flips. At the control rate this would otherwise
        # be one line per step; the transitions are what tell you whether the policy
        # actually learned to assert precision near the microscope. The clamp is
        # included because that is the safety-relevant consequence: an unset
        # max_relative_target means coarse motion is unclamped.
        if precision_active != self._precision_logged:
            clamp = "unclamped" if relative_target is None else f"max {relative_target} deg/step"
            logger.info(
                f"precision mode {'ON' if precision_active else 'OFF'} ({clamp})"
                f"{'' if self._precision_logged is not None else ' [initial]'}"
            )
            self._precision_logged = precision_active

        # Relative interpretation applies ONLY to actions in the policy's action space.
        #
        # This robot is driven by two kinds of caller, and they disagree about what a
        # joint value means. The policy emits per-step DELTAS. The rollout also drives
        # it with ABSOLUTE joint targets, in at least three places that are not
        # obviously related: the reset loop's home teleop (rollout/strategies/
        # episodic.py), the smooth-handover slide and return_to_initial_position
        # (rollout/strategies/core.py). A robot-wide "everything is relative" mode adds
        # the home pose to the current pose, which on 2026-09-16 commanded joint_2 to
        # 81.2 + 10 = 91.2 deg against a (-70, 90) limit and faulted the controller.
        #
        # `precision.state` separates them reliably: it is part of the policy's action
        # space and no homing path emits it (Meca500Home.get_action returns pose keys
        # only, and both core.py slides interpolate over observation pose keys). So the
        # test is "does this action carry every action_features key", not a guess about
        # magnitudes - a delta and an absolute target are not distinguishable by size.
        #
        # Integrate onto the current TARGET pose, not the measured one:
        # GetRtTargetJointPos() is where the controller is driving to, GetJoints() is
        # where the arm has got to. Integrating onto the measured pose would fold servo
        # lag into every step, so a stream of small deltas would fall progressively
        # behind the trajectory it is meant to trace.
        if self.config.relative_actions:
            is_policy_action = set(self.action_features).issubset(action)
            if is_policy_action:
                present_pos_list = self.robot.GetRtTargetJointPos()
                goal_pos = {
                    f"joint_{i + 1}": present_pos_list[i] + goal_pos.get(f"joint_{i + 1}", 0.0)
                    for i in range(6)
                }
            elif self._absolute_action_logged is not True:
                # Loud once, not every step: silently switching interpretation is
                # exactly how the arm ends up somewhere nobody predicted.
                logger.info(
                    f"{self} relative_actions is on, but this action carries only "
                    f"{sorted(action)} and not the full policy action space "
                    f"{sorted(self.action_features)}. Treating it as ABSOLUTE joint "
                    f"targets - this is the homing/reset path, which is correct."
                )
                self._absolute_action_logged = True

        if relative_target is not None:
            present_pos_list = self.robot.GetRtTargetJointPos()
            present_pos = {f"joint_{i + 1}": p for i, p in enumerate(present_pos_list)}
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, relative_target)

        logger.debug(f"Sending goal positions: {goal_pos}")
        self.robot.MoveJoints(
            float(goal_pos["joint_1"]),
            float(goal_pos["joint_2"]),
            float(goal_pos["joint_3"]),
            float(goal_pos["joint_4"]),
            float(goal_pos["joint_5"]),
            float(goal_pos["joint_6"]),
        )
        # Always the ABSOLUTE target that was sent, even in relative_actions mode -
        # this is what the dataset records, and it is what the arm was actually told
        # to do. Note the asymmetry: a rollout recorded with relative_actions=True
        # therefore holds absolute positions, not the deltas the policy emitted, so
        # it is directly comparable with the teleop datasets but is NOT in the same
        # action space the policy was trained on. Re-run the delta conversion on it
        # before using such a rollout for fine-tuning.
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        logger.info("Disconnecting Meca500...")
        try:
            if self.robot.IsConnected():
                # In monitor_mode the wrapper holds an observation-only socket;
                # control commands (PauseMotion/ClearMotion/DeactivateRobot)
                # would raise and trigger mecademicpy's disconnect_on_exception.
                # The teleop owns the control connection and has already
                # deactivated the arm by the time we get here.
                if not self.config.monitor_mode:
                    self.robot.PauseMotion()
                    self.robot.WaitMotionPaused(timeout=5)
                    self.robot.ClearMotion()
                    self.robot.WaitMotionCleared(timeout=5)
                    self.robot.DeactivateRobot()
                    self.robot.WaitDeactivated(timeout=10)
                self.robot.Disconnect()
                self.robot.WaitDisconnected(timeout=5)
        except Exception as e:
            logger.warning(f"Error during disconnect sequence: {e}")

        for cam in self.cameras.values():
            cam.disconnect()

        self._connected = False
        logger.info(f"{self} disconnected.")
