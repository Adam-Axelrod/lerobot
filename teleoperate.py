"""Hand-guidance teleop on the Meca500 with the Bota force sensor.

Workflow: edit the CONFIG block below, then `python teleoperate.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Equivalent to:
    lerobot-teleoperate --robot.type=meca500 --teleop.type=meca500_bota \
        --display_data=true
"""

from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate
from lerobot.teleoperators.meca500_bota.config_meca500_bota import meca500BotaConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
DISPLAY_DATA = False
FPS = 60
# ------------------------------------------------------------------


def main() -> None:
    register_third_party_plugins()

    cfg = TeleoperateConfig(
        robot=Meca500Config(),  # monitor_mode=True (default) — teleop owns activation
        teleop=meca500BotaConfig(),
        fps=FPS,
        display_data=DISPLAY_DATA,
    )

    teleoperate(cfg)


if __name__ == "__main__":
    main()
