from gerf_arms import FOLLOWER_ID, port

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

config = SO101FollowerConfig(
    port=port("follower"),
    id=FOLLOWER_ID,
)
follower = SO101Follower(config)
follower.setup_motors()

