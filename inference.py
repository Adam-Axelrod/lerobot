"""Run policy inference on the Meca500.

Workflow: edit the CONFIG block below, then `python inference.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Equivalent to:
    lerobot-record --robot.type=meca500 --robot.monitor_mode=False \
        --teleop.type=meca500_home --teleop.home=[0,0,0,0,90,0] \
        --policy.path=<USER>/<RUN>_model \
        --dataset.repo_id=<USER>/eval_<RUN> --dataset.single_task=<TASK> \
        --dataset.push_to_hub=False
"""

import shutil
from pathlib import Path

from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
RUN = "red_dot_black_pointer"
TASK = "reach_red_dot"

# ACT inference mode (see note in inference.ps1):
#   None             → full-chunk inference (fast, wobbles at chunk boundaries)
#   float in (0, 1)  → temporal ensembling (smooth, every-step inference)
TEMPORAL_ENSEMBLE_COEFF: float | None = None

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 90.0, 0.0]
NUM_EPISODES = 10
EPISODE_TIME_S = 30
RESET_TIME_S = 10
DISPLAY_DATA = False
# ------------------------------------------------------------------


def main() -> None:
    register_third_party_plugins()

    repo_id = f"{USER}/eval_{RUN}"
    policy_path = f"{USER}/{RUN}_model"

    # Clear any stale local copy so re-runs don't trip the dataset sanity check.
    cache_dir = Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id
    shutil.rmtree(cache_dir, ignore_errors=True)

    policy_overrides: list[str] = []
    if TEMPORAL_ENSEMBLE_COEFF is not None:
        policy_overrides = [
            f"--temporal_ensemble_coeff={TEMPORAL_ENSEMBLE_COEFF}",
            "--n_action_steps=1",
        ]

    policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=policy_overrides)
    policy.pretrained_path = policy_path

    cfg = RecordConfig(
        robot=Meca500Config(monitor_mode=False),
        dataset=DatasetRecordConfig(
            repo_id=repo_id,
            single_task=TASK,
            num_episodes=NUM_EPISODES,
            episode_time_s=EPISODE_TIME_S,
            reset_time_s=RESET_TIME_S,
            push_to_hub=False,
        ),
        teleop=Meca500HomeConfig(home=HOME_JOINTS),
        policy=policy,
        display_data=DISPLAY_DATA,
    )

    record(cfg)


if __name__ == "__main__":
    main()
