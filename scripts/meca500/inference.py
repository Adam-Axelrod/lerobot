"""Run policy inference on the Meca500 — a pushed Hub model or a local checkpoint.

Set CKPT to choose what runs: the finished model from the Hub, the final local
training checkpoint, or a specific earlier one for a mid-training sanity check.

Also the shared checkpoint resolver: microscope_inference.py imports
`resolve_policy` from here rather than duplicating it.

Workflow: edit the CONFIG block below, then `python inference.py`.
Ctrl-C to stop, edit CKPT (or anything else), up-arrow, run again.

Equivalent to:
    lerobot-rollout --strategy.type=episodic \
        --robot.type=meca500 --robot.monitor_mode=False \
        --teleop.type=meca500_home --teleop.home=[0,0,0,0,90,0] \
        --policy.path=<USER>/<RUN>_model            # or a local checkpoint dir \
        --dataset.repo_id=<USER>/rollout_<RUN> \
        --dataset.single_task=<TASK> \
        --dataset.push_to_hub=False
"""

import sys
from pathlib import Path

from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.rollout.configs import EpisodicStrategyConfig, RolloutConfig
from lerobot.scripts.lerobot_rollout import rollout
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_plugins

# Sentinel for CKPT meaning "the pushed Hub model" rather than a local checkpoint.
HUB = "hub"

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
RUN = "space_mouse_puple_dot_100demos"
TASK = "reach_purple_dot"

# Which policy to run:
#   HUB        → the pushed Hub model <USER>/<RUN>_model
#   None       → the FINAL local checkpoint under outputs/train/<RUN>/checkpoints/
#   "025000"   → that specific local checkpoint (mid-training sanity check)
CKPT: str | None = HUB

# ACT inference mode:
#   None             → full-chunk inference (fast, wobbles at chunk boundaries)
#   float in (0, 1)  → temporal ensembling (smooth, every-step inference)
TEMPORAL_ENSEMBLE_COEFF: float | None = None

HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 90.0, 0.0]
NUM_EPISODES = 10
EPISODE_TIME_S = 30
RESET_TIME_S = 10
FPS = 30
DISPLAY_DATA = True
# ------------------------------------------------------------------


def resolve_policy(user: str, run: str, ckpt: str | None) -> tuple[str, str | Path]:
    """Resolve the requested policy into ``(label, path)``.

    ``ckpt`` selects the source:

      * ``HUB``      -- the pushed Hub model ``<user>/<run>_model``
      * ``None``     -- the final (highest-numbered) local checkpoint under
        ``outputs/train/<run>/checkpoints/``
      * ``"025000"`` -- that specific local checkpoint

    ``label`` is appended to the rollout dataset's repo_id so evaluating two
    checkpoints of the same run does not have the second silently overwrite the
    first. It is empty for the Hub model, preserving the plain ``rollout_<run>``
    name that finished models already use.

    Raises:
        FileNotFoundError: if the run has no checkpoints, or the requested step
            does not exist — caught here rather than surfacing later as an
            opaque load error.
    """
    if ckpt == HUB:
        return "", f"{user}/{run}_model"

    checkpoints_dir = Path("outputs/train") / run / "checkpoints"

    if ckpt is None:
        steps = sorted(d.name for d in checkpoints_dir.glob("*") if d.is_dir() and d.name.isdigit())
        if not steps:
            raise FileNotFoundError(f"No checkpoints found under {checkpoints_dir}. Check RUN={run!r}.")
        ckpt = steps[-1]
        print(f"[ckpt] CKPT=None -> using final checkpoint {ckpt}.")

    policy_path = checkpoints_dir / ckpt / "pretrained_model"
    if not policy_path.is_dir():
        raise FileNotFoundError(
            f"Checkpoint not found: {policy_path}. "
            f"Check RUN={run!r} and CKPT={ckpt!r} (zero-padded step number)."
        )
    return ckpt, policy_path


def main() -> None:
    register_third_party_plugins()

    label, policy_path = resolve_policy(USER, RUN, CKPT)

    policy_overrides: list[str] = []
    if TEMPORAL_ENSEMBLE_COEFF is not None:
        policy_overrides = [
            f"--temporal_ensemble_coeff={TEMPORAL_ENSEMBLE_COEFF}",
            "--n_action_steps=1",
        ]

    # Strip CLI args before constructing RolloutConfig so __post_init__ doesn't
    # re-parse them and overwrite the policy we're about to load.
    sys.argv = [sys.argv[0]]

    policy = PreTrainedConfig.from_pretrained(policy_path, cli_overrides=policy_overrides)
    policy.pretrained_path = policy_path

    cfg = RolloutConfig(
        robot=Meca500Config(id="meca500", monitor_mode=False),
        dataset=DatasetRecordConfig(
            repo_id=f"{USER}/rollout_{RUN}" + (f"_{label}" if label else ""),
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

    try:
        rollout(cfg)
    except DeviceNotConnectedError as e:
        sys.exit(f"\nERROR: {e}")


if __name__ == "__main__":
    main()
