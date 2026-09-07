"""Train an imitation-learning policy on a recorded Meca500 microscope dataset.

Point this at a LeRobot dataset on the Hugging Face Hub (e.g. the
`microscope_pipette` set recorded with record_microscope.py), pick an
architecture (ACT, Diffusion, VQ-BeT, ...), and go. Edit the CONFIG block
below, then `python train_microscope.py`.

Predicting `precision.state`
----------------------------
`precision.state` is not a separate output head — it is the 7th channel of the
`action` vector (6 joint targets + the latched precision-mode flag; see
Meca500.action_features). Every policy here regresses the *whole* action vector,
so the precision toggle is learned automatically alongside the joints, with no
special wiring. On startup this script inspects the dataset and prints the
precision channel's stats so you can confirm it's present and actually toggles
(min ~0, max ~1). If your randomly-chosen SUBSET happens to contain only coarse
demos, the flag never fires and the model can't learn it — the startup check
warns loudly if the full dataset never reaches precision.state=1.

Subset training
---------------
SUBSET_FRACTION < 1.0 trains on a random subset of whole episodes (seeded by
SUBSET_SEED for reproducibility). Frames are kept episode-contiguous — we sample
episodes, never individual frames — so action chunking stays valid.

Checkpoints land in outputs/train/<RUN_NAME>/checkpoints/<step>/pretrained_model,
which is exactly where microscope_inference.py looks for them.

Requires: pip install 'lerobot[training]'  (accelerate + wandb extras).
"""

import random
import re
import sys
from pathlib import Path

# Import the policies package so every built-in architecture registers itself
# with PreTrainedConfig's choice registry (act, diffusion, vqbet, tdmpc, ...).
import lerobot.policies  # noqa: F401
from lerobot.configs.default import DatasetConfig, WandBConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.scripts.lerobot_train import train
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
# --- Run identity ---
# Names this training run. Two things derive from it, so change it for every new
# run or the script will refuse to start:
#   * checkpoints -> outputs/train/<RUN_NAME>/checkpoints/<step>/pretrained_model
#     (RESUME=False refuses to start if that directory already exists)
#   * the pushed model repo -> <HF_USER>/<RUN_NAME>_<ARCHITECTURE>, which is
#     OVERWRITTEN if it already exists on the Hub
# Keep it stable only while resuming that same run. microscope_inference.py's RUN
# must match this to evaluate the resulting checkpoints.
RUN_NAME = "microscope_pipette_2026-09-02"
RESUME = False  # True to continue an interrupted run in the same output dir

# --- Dataset (Hugging Face Hub repo_id, or a local dataset cached under HF_LEROBOT_HOME) ---
USER = "AdamAxelrod"
NAME = "microscope_pipette_2026-09-02"
TASK = "move_pipette_under_microscope"  # informational; used for the run/model name

# --- Architecture ---
# Any registered policy type: "act", "diffusion", "vqbet", "tdmpc", "pi0", "smolvla", ...
# ACT is the usual choice for this kind of visuomotor manipulation dataset.
ARCHITECTURE = "act"

# Per-architecture hyper-parameter overrides (optional). Left empty, each policy
# uses its own sensible training preset. Examples for ACT:
#   POLICY_KWARGS = {"chunk_size": 100, "n_action_steps": 100, "dim_model": 512}
POLICY_KWARGS: dict = {}

# --- Random subset of the dataset ---
# 1.0 = train on every episode. e.g. 0.5 = a random half of the episodes.
SUBSET_FRACTION = 1.0
SUBSET_SEED = 42  # controls *which* episodes are drawn; independent of TRAIN_SEED

# --- Training loop ---
STEPS = 100_000  # total optimizer steps
BATCH_SIZE = 8
SAVE_FREQ = 5_000  # checkpoint every N steps (and at the end)
LOG_FREQ = 100
NUM_WORKERS = 4  # dataloader workers; set 0 on Windows if you hit spawn/pickling errors
TRAIN_SEED = 1000

# --- Hardware ---
DEVICE = "cuda"  # "cuda", "cuda:0", "cpu", or "mps". Accelerate auto-detects the GPU.

# Video decoder for reading the dataset's camera streams. "pyav" bundles its own
# FFmpeg and works out of the box; "torchcodec" is faster but needs FFmpeg
# "full-shared" DLLs on PATH (not installed on this machine — leave "pyav").
VIDEO_BACKEND = "pyav"

# --- Output / logging ---
# (RUN_NAME / RESUME live at the top of this block — they name the run.)
PUSH_TO_HUB = True  # push the trained policy to the Hub at the end
HF_USER = "AdamAxelrod"  # only used to build the model repo_id when pushing

USE_WANDB = True
WANDB_PROJECT = "meca500_microscope"
# Which wandb run a RESUME should reattach to. None = auto-detect the most recent
# run under this run's output dir (see find_wandb_run_id). Set explicitly to pin one.
WANDB_RUN_ID: str | None = None
# ------------------------------------------------------------------


def build_episode_subset(repo_id: str) -> tuple[list[int] | None, int]:
    """Return a sorted random subset of episode indices (or None for all episodes).

    Reads the dataset metadata to learn the total episode count, then draws a
    reproducible random subset of whole episodes using SUBSET_SEED. Returns
    (episodes, total_episodes); `episodes=None` means "use the whole dataset".
    """
    meta = LeRobotDatasetMetadata(repo_id)
    total = meta.total_episodes

    _report_precision_channel(meta)

    if SUBSET_FRACTION >= 1.0:
        print(f"[subset] Using all {total} episodes.")
        return None, total

    if not 0.0 < SUBSET_FRACTION < 1.0:
        raise ValueError(f"SUBSET_FRACTION must be in (0, 1], got {SUBSET_FRACTION}")

    k = max(1, round(total * SUBSET_FRACTION))
    rng = random.Random(SUBSET_SEED)
    episodes = sorted(rng.sample(range(total), k))
    print(
        f"[subset] Training on {k}/{total} episodes "
        f"(fraction={SUBSET_FRACTION}, seed={SUBSET_SEED}): {episodes}"
    )
    if k < 3:
        print(
            "[subset] WARNING: very few episodes selected — the model may not see a "
            "precision.state toggle. Raise SUBSET_FRACTION if precision isn't learned."
        )
    return episodes, total


def _report_precision_channel(meta: LeRobotDatasetMetadata) -> None:
    """Print the precision.state channel's location/stats and warn if it never toggles."""
    action_feature = meta.features.get("action", {})
    names = action_feature.get("names")
    if not names or "precision.state" not in names:
        print(
            "[precision] WARNING: no 'precision.state' channel found in the dataset's "
            f"action feature (names={names}). This dataset may not have been recorded "
            "with record_microscope.py — the precision flag can't be learned."
        )
        return

    idx = list(names).index("precision.state")
    stats = meta.stats.get("action", {})
    lo = float(stats["min"][idx]) if "min" in stats else float("nan")
    hi = float(stats["max"][idx]) if "max" in stats else float("nan")
    mean = float(stats["mean"][idx]) if "mean" in stats else float("nan")
    print(
        f"[precision] 'precision.state' is action channel {idx}/{len(names)} — "
        f"learned as part of the action vector. Full-dataset stats: "
        f"min={lo:.3f} max={hi:.3f} mean={mean:.3f}."
    )
    if hi < 0.5:
        print(
            "[precision] WARNING: precision.state never reaches 1 anywhere in the dataset. "
            "There are no precision-mode frames to learn from — check your recordings."
        )


def find_wandb_run_id(run_name: str) -> str | None:
    """The most recent wandb run id under this run's output dir, or None.

    Needed on Windows. When resuming, WandBLogger falls back to
    get_wandb_run_id_from_filesystem(), which globs ``wandb/latest-run/run-*`` --
    but ``latest-run`` is a symlink wandb does not create on Windows, and that
    helper also splits paths on "/" which Windows globs never contain. Either way
    it raises "Couldn't get the previous WandB run ID for run resumption."

    Passing ``run_id`` explicitly skips that lookup entirely, so this reads the id
    straight off the run directory name (``run-<timestamp>-<id>/run-<id>.wandb``),
    picking the newest by mtime if a run dir has accumulated more than one.
    """
    wandb_dir = Path("outputs/train") / run_name / "wandb"
    runs = sorted(wandb_dir.glob("run-*/run-*.wandb"), key=lambda p: p.stat().st_mtime)
    if not runs:
        return None
    match = re.fullmatch(r"run-(.+)\.wandb", runs[-1].name)
    return match.group(1) if match else None


def build_policy_config() -> PreTrainedConfig:
    """Instantiate the chosen policy config, leaving input/output features to be
    inferred from the dataset (so the full 7-dim action, incl. precision.state, is used)."""
    try:
        policy_cls = PreTrainedConfig.get_choice_class(ARCHITECTURE)
    except KeyError as e:
        known = ", ".join(sorted(PreTrainedConfig.get_known_choices()))
        raise ValueError(
            f"Unknown ARCHITECTURE {ARCHITECTURE!r}. Registered policies: {known}."
        ) from e

    model_repo_id = f"{HF_USER}/{RUN_NAME}_{ARCHITECTURE}"
    # input_features/output_features left as the empty defaults => inferred from
    # the dataset in make_policy, which includes every action channel.
    return policy_cls(
        device=DEVICE,
        push_to_hub=PUSH_TO_HUB,
        repo_id=model_repo_id,  # always set: required by validate() even when not pushing
        **POLICY_KWARGS,
    )


def main() -> None:
    register_third_party_plugins()

    episodes, _ = build_episode_subset(f"{USER}/{NAME}")

    # Only relevant when resuming: wandb reattaches to the existing run rather than
    # starting a second one for the same training run.
    wandb_run_id = WANDB_RUN_ID
    if USE_WANDB and RESUME and wandb_run_id is None:
        wandb_run_id = find_wandb_run_id(RUN_NAME)
        if wandb_run_id:
            print(f"[wandb] resuming run {wandb_run_id}")
        else:
            print(
                f"[wandb] no previous run found under outputs/train/{RUN_NAME}/wandb — "
                f"resume will fail. Set WANDB_RUN_ID, or USE_WANDB=False for this run."
            )

    cfg = TrainPipelineConfig(
        dataset=DatasetConfig(
            repo_id=f"{USER}/{NAME}",
            episodes=episodes,
            video_backend=VIDEO_BACKEND,
        ),
        policy=build_policy_config(),
        output_dir=f"outputs/train/{RUN_NAME}",
        job_name=RUN_NAME,
        resume=RESUME,
        seed=TRAIN_SEED,
        num_workers=NUM_WORKERS,
        batch_size=BATCH_SIZE,
        steps=STEPS,
        save_freq=SAVE_FREQ,
        log_freq=LOG_FREQ,
        wandb=WandBConfig(enable=USE_WANDB, project=WANDB_PROJECT, run_id=wandb_run_id),
    )

    # train() is @parser.wrap()-decorated: passing a TrainPipelineConfig instance
    # as the first arg makes it skip CLI parsing and use our config directly.
    # Clear argv so nothing downstream (validate() re-parses it) sees stray flags.
    sys.argv = [sys.argv[0]]

    if RESUME:
        # validate() applies the policy's optimizer/scheduler presets only when NOT
        # resuming, because upstream's resume path expects the whole config to have
        # been *loaded from* the checkpoint's train_config.json by draccus. This
        # script builds the config in Python instead, so on resume nothing populates
        # them and make_optimizer_and_scheduler() raises "Optimizer config is
        # required". Apply the presets ourselves — they are what the original run
        # used (verified field-for-field against the checkpoint's train_config.json),
        # and the optimizer *state* is restored separately from
        # training_state/optimizer_state.safetensors.
        cfg.optimizer = cfg.policy.get_optimizer_preset()
        cfg.scheduler = cfg.policy.get_scheduler_preset()

        # Resuming cannot be expressed on the config object alone: validate() reads
        # --config_path straight out of sys.argv (draccus has already consumed it by
        # then), so `resume=True` without it raises "A config_path is expected when
        # resuming a run". Point it at the last checkpoint's train_config.json.
        resume_config = (
            Path("outputs/train") / RUN_NAME / "checkpoints" / "last" / "pretrained_model" / "train_config.json"
        )
        if not resume_config.is_file():
            raise FileNotFoundError(
                f"RESUME=True but no checkpoint to resume from: {resume_config} does not exist. "
                f"Check RUN_NAME={RUN_NAME!r}, or set RESUME=False to start a fresh run "
                f"(delete outputs/train/{RUN_NAME}/ first, since a fresh run refuses to "
                f"overwrite an existing output dir)."
            )
        print(f"[resume] continuing from {resume_config}")
        sys.argv.append(f"--config_path={resume_config}")

    train(cfg)


if __name__ == "__main__":
    main()
