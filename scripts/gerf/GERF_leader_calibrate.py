from gerf_arms import LEADER_CALIBRATION_DIR, LEADER_ID, port

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

config = SO101LeaderConfig(
    port=port("leader"),
    id=LEADER_ID,
    calibration_dir=LEADER_CALIBRATION_DIR,
)

leader = SO101Leader(config)
leader.connect(calibrate=False)
leader.calibrate()
leader.disconnect()
