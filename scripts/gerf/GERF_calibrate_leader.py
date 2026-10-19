from gerf_arms import LEADER_ID, port

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

config = SO101LeaderConfig(
    port=port("leader"),
    id=LEADER_ID,
)
leader = SO101Leader(config)
leader.setup_motors()
