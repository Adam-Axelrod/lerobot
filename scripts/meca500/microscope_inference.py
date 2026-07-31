"""Run policy inference on the Meca500 microscope/pipette rig.

The general inference script for the microscope rig (overhead + wrist + microscope
cameras) and the 7-dim action space (6 joints + the predicted `precision.state`).
It runs any trained policy on the arm — whether you're validating an intermediate
training checkpoint or deploying the final model for real use. By default it loads
a local checkpoint from outputs/train/<RUN>/checkpoints/<CKPT>/; point CKPT at the
final step to run your finished model, or an earlier step to compare checkpoints.

When the policy's predicted `precision.state` crosses 0.5, Meca500.send_action clamps
per-step joint motion to PRECISION_MAX_REL_TARGET instead of MAX_REL_TARGET, so the
arm "scales down its own movements" under the microscope. Tune both (values are
joint-space degrees per step).

Workflow: edit the CONFIG block below, then `python microscope_inference.py`.
Ctrl-C to stop, edit CKPT (or anything else), up-arrow, run again.

Equivalent to:
    lerobot-rollout --strategy.type=episodic \
        --robot.type=meca500_microscope --robot.monitor_mode=False \
        --robot.precision_max_relative_target=<PRECISION_MAX_REL_TARGET> \
        --teleop.type=meca500_home --teleop.home=[0,0,0,0,90,0] \
        --policy.path=outputs/train/<RUN>/checkpoints/<CKPT>/pretrained_model \
        --dataset.repo_id=<USER>/rollout_<RUN>_<CKPT> \
        --dataset.single_task=<TASK> --dataset.push_to_hub=False
"""

import sys
from pathlib import Path

from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig
from lerobot.rollout.configs import EpisodicStrategyConfig, RolloutConfig
from lerobot.scripts.lerobot_rollout import rollout
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
RUN = "microscope_pipette"
# None → automatically use the final (highest-numbered) checkpoint. Or set a
# zero-padded step number (e.g. "025000") to pin an earlier one for comparison.
CKPT: str | None = None
TASK = "move_pipette_under_microscope"

# ACT inference mode:
#   None             → full-chunk inference (fast, wobbles at chunk boundaries)
#   float in (0, 1)  → temporal ensembling (smooth, every-step inference)
TEMPORAL_ENSEMBLE_COEFF: float | None = None

# Per-step joint-delta clamps (degrees). None = unclamped.
#   MAX_REL_TARGET           applies when precision is off (coarse approach)
#   PRECISION_MAX_REL_TARGET applies when the policy predicts precision.state > 0.5
# Tune after the first checkpoint — start conservative to protect the pipette.
MAX_REL_TARGET: float | None = None
PRECISION_MAX_REL_TARGET: float | None = 0.5

HOME_JOINTS = [70, 10, 10, 90, -80, 15]
NUM_EPISODES = 10
EPISODE_TIME_S = 30
RESET_TIME_S = 10
FPS = 20
DISPLAY_DATA = True
# ------------------------------------------------------------------


def resolve_checkpoint(run: str, ckpt: str | None) -> tuple[str, Path]:
    """Resolve CKPT to a concrete checkpoint dir, defaulting to the final one.

    ckpt=None picks the highest-numbered step under outputs/train/<run>/checkpoints/;
    otherwise the given step is used verbatim. Returns (step, pretrained_model_path).
    """
    checkpoints_dir = Path("outputs/train") / run / "checkpoints"

    if ckpt is None:
        steps = sorted(
            d.name for d in checkpoints_dir.glob("*") if d.is_dir() and d.name.isdigit()
        )
        if not steps:
            raise FileNotFoundError(
                f"No checkpoints found under {checkpoints_dir}. Check RUN='{run}'."
            )
        ckpt = steps[-1]
        print(f"[ckpt] CKPT=None → using final checkpoint {ckpt}.")

    policy_path = checkpoints_dir / ckpt / "pretrained_model"
    if not policy_path.is_dir():
        raise FileNotFoundError(
            f"Checkpoint not found: {policy_path}. "
            f"Check RUN='{run}' and CKPT='{ckpt}' (zero-padded step number)."
        )
    return ckpt, policy_path


def main() -> None:
    register_third_party_plugins()

    ckpt, policy_path = resolve_checkpoint(RUN, CKPT)

    policy_overrides: list[str] = []
    if TEMPORAL_ENSEMBLE_COEFF is not None:
        policy_overrides = [
            f"--temporal_ensemble_coeff={TEMPORAL_ENSEMBLE_COEFF}",
            "--n_action_steps=1",
        ]

    # Strip CLI args before constructing RolloutConfig so __post_init__ doesn't
    # re-parse them and overwrite the local checkpoint policy we're about to load.
    sys.argv = [sys.argv[0]]

    policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=policy_overrides)
    policy.pretrained_path = policy_path

    cfg = RolloutConfig(
        robot=Meca500MicroscopeConfig(
            id="meca500_microscope",
            monitor_mode=False,
            max_relative_target=MAX_REL_TARGET,
            precision_max_relative_target=PRECISION_MAX_REL_TARGET,
        ),
        dataset=DatasetRecordConfig(
            repo_id=f"{USER}/rollout_{RUN}_{ckpt}",
            single_task=TASK,
            fps=FPS,
            num_episodes=NUM_EPISODES,
            episode_time_s=EPISODE_TIME_S,
            reset_time_s=RESET_TIME_S,
            push_to_hub=False,
        ),
        teleop=Meca500HomeConfig(id="meca500_home", home=HOME_JOINTS),
        policy=policy,
        strategy=EpisodicStrategyConfig(),
        fps=FPS,
        display_data=DISPLAY_DATA,
    )

    rollout(cfg)


if __name__ == "__main__":
    main()
