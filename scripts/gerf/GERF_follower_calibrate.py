from gerf_arms import FOLLOWER_CALIBRATION_DIR, FOLLOWER_ID, port

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

config = SO101FollowerConfig(
    port=port("follower"),
    id=FOLLOWER_ID,
    calibration_dir=FOLLOWER_CALIBRATION_DIR,
)

follower = SO101Follower(config)
follower.connect(calibrate=False)
follower.calibrate()
follower.disconnect()
