from unittest.mock import MagicMock

import numpy as np

from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500.meca500 import Meca500


class FakeCamera:
    def __init__(self, name: str, should_fail: bool):
        self.name = name
        self.should_fail = should_fail
        self.is_connected = False
        self.height = 120
        self.width = 160

    def connect(self):
        if self.should_fail:
            raise ConnectionError(f"Fake camera {self.name} unavailable")
        self.is_connected = True

    def async_read(self):
        if not self.is_connected:
            raise RuntimeError(f"Fake camera {self.name} is not connected")
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def read_latest(self, max_age_ms: int = 500):
        """Non-blocking peek, as Camera.read_latest defines it.

        Meca500.get_observation() prefers this over async_read() so the control
        loop never blocks waiting for the next frame. The real Camera base class
        provides read_latest for every implementation, so a fake standing in for
        one has to offer it too or the robot falls back to async_read for the
        wrong reason.
        """
        return self.async_read()


def _mock_robot(config, present):
    """A connected Meca500 whose controller reports `present` as its target pose."""
    robot = Meca500(config)
    robot.cameras = {}
    robot.robot = MagicMock()
    robot.robot.GetRtTargetJointPos = MagicMock(return_value=list(present))
    robot._connected = True
    return robot


def test_send_action_absolute_mode_passes_targets_through():
    robot = _mock_robot(Meca500Config(monitor_mode=False), [10.0] * 6)
    action = {f"joint_{i + 1}.pos": 25.0 for i in range(6)}

    sent = robot.send_action(action)

    assert robot.robot.MoveJoints.call_args[0] == (25.0,) * 6
    assert sent["joint_1.pos"] == 25.0


def test_send_action_relative_mode_integrates_onto_target_pose():
    """A delta action must be added to where the controller is driving to.

    This is the deployment half of the delta-action conversion: the policy emits
    per-step increments, and the robot turns them back into absolute MoveJoints
    targets. If these two halves disagree the arm traces a different trajectory
    from the demonstrated one, silently.
    """
    present = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    robot = _mock_robot(Meca500Config(monitor_mode=False, relative_actions=True), present)
    action = {f"joint_{i + 1}.pos": 0.5 for i in range(6)}

    sent = robot.send_action(action)

    expected = tuple(p + 0.5 for p in present)
    assert robot.robot.MoveJoints.call_args[0] == expected
    # send_action always reports the ABSOLUTE target it sent, not the delta it got.
    assert sent["joint_3.pos"] == 30.5


def test_send_action_relative_mode_zero_delta_holds_position():
    """Zero delta means hold -- this is what the last frame of every episode encodes."""
    present = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    robot = _mock_robot(Meca500Config(monitor_mode=False, relative_actions=True), present)

    robot.send_action({f"joint_{i + 1}.pos": 0.0 for i in range(6)})

    assert robot.robot.MoveJoints.call_args[0] == tuple(present)


def test_send_action_relative_mode_clamp_bounds_the_delta():
    """With relative actions the clamp becomes a direct bound on the step size."""
    present = [0.0] * 6
    robot = _mock_robot(
        Meca500Config(monitor_mode=False, relative_actions=True, max_relative_target=0.06), present
    )

    robot.send_action({f"joint_{i + 1}.pos": 5.0 for i in range(6)})

    assert robot.robot.MoveJoints.call_args[0] == (0.06,) * 6


def test_send_action_relative_mode_ignores_precision_gate_as_a_joint():
    """precision.state is a gate, never differenced and never integrated."""
    present = [7.0] * 6
    robot = _mock_robot(Meca500Config(monitor_mode=False, relative_actions=True), present)

    sent = robot.send_action({**{f"joint_{i + 1}.pos": 0.1 for i in range(6)}, "precision.state": 1.0})

    assert robot.robot.MoveJoints.call_args[0] == (7.1,) * 6
    assert "precision.state" not in sent


def test_meca500_connect_with_partial_camera_failure():
    config = Meca500Config()

    robot = Meca500(config)
    robot.cameras = {
        "good_cam": FakeCamera("good_cam", should_fail=False),
        "bad_cam": FakeCamera("bad_cam", should_fail=True),
    }

    # Avoid requiring a real Meca500 hardware connection in unit tests.
    robot.robot = MagicMock()
    robot.robot.Connect = MagicMock()
    robot.robot.ActivateAndHome = MagicMock()
    robot.robot.WaitHomed = MagicMock()

    robot.connect()

    assert robot.is_connected

    obs = robot.get_observation()
    assert "good_cam" in obs
    assert "bad_cam" in obs
    assert obs["good_cam"].shape == (120, 160, 3)
    assert obs["bad_cam"].shape == (120, 160, 3)

    # Confirm that the invalid camera gives a valid blank frame as fallback
    assert np.count_nonzero(obs["bad_cam"]) == 0
