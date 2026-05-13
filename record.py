"""Record a teleop dataset on the Meca500 with the Bota force-sensor hand-guidance.

Workflow: edit the CONFIG block below, then `python record.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Equivalent to:
    lerobot-record --robot.type=meca500 --teleop.type=meca500_bota \
        --dataset.repo_id=<USER>/<NAME> --dataset.single_task=<TASK> \
        --dataset.num_episodes=N --dataset.fps=30 \
        --dataset.episode_time_s=60 --dataset.reset_time_s=60
"""

import shutil
from pathlib import Path

from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.teleoperators.meca500_bota.config_meca500_bota import meca500BotaConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
NAME = "red_dot_black_pointer"
TASK = "reach_red_dot"

NUM_EPISODES = 50
FPS = 30
EPISODE_TIME_S = 60
RESET_TIME_S = 60
DISPLAY_DATA = True
PUSH_TO_HUB = True
# ------------------------------------------------------------------


def main() -> None:
    register_third_party_plugins()

    repo_id = f"{USER}/{NAME}"

    # Clear any stale local copy so re-runs don't trip the dataset sanity check.
    cache_dir = Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id
    shutil.rmtree(cache_dir, ignore_errors=True)

    cfg = RecordConfig(
        robot=Meca500Config(),  # monitor_mode=True (default) — teleop owns activation
        dataset=DatasetRecordConfig(
            repo_id=repo_id,
            single_task=TASK,
            num_episodes=NUM_EPISODES,
            fps=FPS,
            episode_time_s=EPISODE_TIME_S,
            reset_time_s=RESET_TIME_S,
            push_to_hub=PUSH_TO_HUB,
        ),
        teleop=meca500BotaConfig(),
        display_data=DISPLAY_DATA,
    )

    record(cfg)


if __name__ == "__main__":
    main()
