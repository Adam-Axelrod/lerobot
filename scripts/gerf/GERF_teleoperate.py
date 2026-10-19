from gerf_arms import (
    FOLLOWER_CALIBRATION_DIR,
    FOLLOWER_ID,
    LEADER_CALIBRATION_DIR,
    LEADER_ID,
    port,
)

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

robot_config = SO101FollowerConfig(
    port=port("follower"),
    id=FOLLOWER_ID,
    calibration_dir=FOLLOWER_CALIBRATION_DIR,
)

teleop_config = SO101LeaderConfig(
    port=port("leader"),
    id=LEADER_ID,
    calibration_dir=LEADER_CALIBRATION_DIR,
)

robot = SO101Follower(robot_config)
teleop_device = SO101Leader(teleop_config)
robot.connect()
teleop_device.connect()

while True:
    action = teleop_device.get_action()
    robot.send_action(action)
