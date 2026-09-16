"""Drive the Meca500 to a fixed home pose, then disconnect.

Workflow: set RIG below to the bench you are on, then `python reset.py`. The pose itself
comes from HOME_JOINTS in rigs.py, shared with record and teleop.

Useful for checking a NEW bench's home pose is actually clear of the microscope and the
pipette holder before committing to a run - record drives here between every episode.

Uses the `meca500_home` teleop to source the target pose and `Meca500.send_action`
to execute it. The Meca500 wrapper is opened with `monitor_mode=False` so it
owns activation/homing this time (unlike during teleop/record where the
Bota teleop owns the control connection).
"""

import logging
import time

from rigs import require_home_joints, rig_config

from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.teleoperators.meca500_home.meca500_home import Meca500Home
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging

# ----------------------------- CONFIG -----------------------------
# Which bench you are on. The pose itself lives per-rig in rigs.py, so this script and
# record/teleop cannot disagree about where "home" is - they used to each keep their own
# copy, and driving to another bench's pose parks the arm somewhere on a DIFFERENT desk.
RIG = "meca500_basler"

MOVE_TIMEOUT_S = 30.0  # max wait for the arm to reach the home pose
# ------------------------------------------------------------------


def main() -> None:
    init_logging()
    register_third_party_plugins()

    # Refuses (with instructions) if this bench has no measured pose, rather than
    # silently driving to another rig's.
    home = require_home_joints(RIG)

    # Robot in control mode (not monitor) so it activates, homes, and accepts MoveJoints.
    # No cameras needed for a reset — drop them to skip OpenCV startup.
    robot = make_robot_from_config(rig_config(RIG, monitor_mode=False, cameras={}))
    teleop = Meca500Home(Meca500HomeConfig(id="meca500_home", home=home))

    robot.connect()       # activates + homes to factory zero
    robot.configure()     # sets joint velocity / blending defaults
    teleop.connect()

    try:
        action = teleop.get_action()
        logging.info(f"Driving to home: {action}")
        robot.send_action(action)

        # Block until the motion queue drains; otherwise robot.disconnect() will
        # ClearMotion() and abort the move mid-flight.
        deadline = time.perf_counter() + MOVE_TIMEOUT_S
        robot.robot.WaitIdle(timeout=MOVE_TIMEOUT_S)
        if time.perf_counter() > deadline:
            logging.warning("WaitIdle returned past deadline — move may not have completed.")
        else:
            logging.info("Home pose reached.")
    finally:
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
