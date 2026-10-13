import logging

import numpy as np

from ..teleoperator import Teleoperator
from .config_meca500_scripted import Meca500ScriptedConfig

logger = logging.getLogger(__name__)

JOINT_KEYS = [f"joint_{i + 1}.pos" for i in range(6)]


class Meca500Scripted(Teleoperator):
    """Replays a pre-planned joint trajectory, one row per control step.

    The automated recorder's "operator": it has no hardware, and emits ABSOLUTE joint targets
    that the robot (in control mode, ``monitor_mode=False``) executes with ``MoveJoints`` -
    the same path a deployed policy drives. So the recorded action is the command itself,
    not a position readback as with the SpaceMouse.

    The action carries ``precision.state`` as well as the joints, matching
    ``Meca500.action_features`` exactly, so recorded datasets share the SpaceMouse schema.
    Once the plan is exhausted it holds the last row, so a record_loop that runs a frame or
    two past the plan keeps the arm still rather than erroring.
    """

    config_class = Meca500ScriptedConfig
    name = "meca500_scripted"

    def __init__(self, config: Meca500ScriptedConfig):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._joints: np.ndarray | None = None
        self._precision: np.ndarray | None = None
        self._index = 0

    @property
    def action_features(self) -> dict[str, type]:
        return {**dict.fromkeys(JOINT_KEYS, float), "precision.state": float}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        self._connected = True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def load_plan(self, joints, precision=None) -> None:
        """Load a (N, 6) joint trajectory in degrees and an optional (N,) precision flag."""
        joints = np.asarray(joints, dtype=float)
        if joints.ndim != 2 or joints.shape[1] != 6 or len(joints) == 0:
            raise ValueError(f"expected a non-empty (N, 6) joint array, got shape {joints.shape}")
        precision = np.zeros(len(joints)) if precision is None else np.asarray(precision, dtype=float)
        if precision.shape != (len(joints),):
            raise ValueError(f"precision must have shape ({len(joints)},), got {precision.shape}")
        self._joints, self._precision, self._index = joints, precision, 0

    @property
    def remaining(self) -> int:
        return 0 if self._joints is None else max(len(self._joints) - self._index, 0)

    @property
    def done(self) -> bool:
        return self.remaining == 0

    def get_action(self) -> dict[str, float]:
        if self._joints is None:
            raise RuntimeError("Meca500Scripted.get_action() called before load_plan().")
        i = min(self._index, len(self._joints) - 1)
        self._index += 1
        action = {k: float(v) for k, v in zip(JOINT_KEYS, self._joints[i], strict=True)}
        action["precision.state"] = float(self._precision[i])
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False
