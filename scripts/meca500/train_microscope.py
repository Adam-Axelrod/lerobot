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

Action representation
---------------------
`DELTA_ACTIONS` declares whether the dataset's action channel holds per-step joint
DELTAS or absolute joint targets, and the startup check fails the run if the dataset
disagrees. Deltas exist because the SpaceMouse teleop records a position readback
rather than a command (`get_action()` returns `GetJoints()`), so an absolute dataset's
action equals its own observation.state in 97.4% of frames -- a policy trained on it
learns to predict where the arm already is, and commands no motion. Build a delta
dataset with scripts/meca500/convert_to_delta_actions.py, and deploy the resulting
checkpoint with `Meca500Config(relative_actions=True)`.

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

import numpy as np

# Import the policies package so every built-in architecture registers itself
# with PreTrainedConfig's choice registry (act, diffusion, vqbet, tdmpc, ...).
import lerobot.policies  # noqa: F401
from lerobot.configs.default import DatasetConfig, WandBConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata
from lerobot.scripts.lerobot_train import train
from lerobot.transforms import ImageTransformsConfig
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
RUN_NAME = "basler_pipette_2026-09-16_delta"
RESUME = False  # True to continue an interrupted run in the same output dir

# --- Dataset (Hugging Face Hub repo_id, or a local dataset cached under HF_LEROBOT_HOME) ---
USER = "AdamAxelrod"
NAME = "basler_pipette_2026-09-16_delta"
TASK = "move_pipette_under_microscope"  # informational; used for the run/model name

# What the dataset's action channel means. This is a DECLARATION of intent, checked
# against the dataset at startup (see _report_action_representation) - it does not
# convert anything. Build a delta dataset with:
#
#     python scripts/meca500/convert_to_delta_actions.py
#
#   True  -> actions are per-step joint DELTAS. Deploy with
#            Meca500Config(relative_actions=True).
#   False -> actions are absolute joint targets (the original recordings).
#
# Getting this pair wrong is the one failure mode that bites hard: a delta-trained
# policy deployed with relative_actions=False sends ~0.005 deg ABSOLUTE joint targets
# and drives the arm into its lower limits.
DELTA_ACTIONS = True

# --- Architecture ---
# Any registered policy type: "act", "diffusion", "vqbet", "tdmpc", "pi0", "smolvla", ...
# ACT is the usual choice for this kind of visuomotor manipulation dataset.
ARCHITECTURE = "act"

# Per-architecture hyper-parameter overrides (optional). Left empty, each policy
# uses its own sensible training preset. Examples for ACT:
#   POLICY_KWARGS = {"chunk_size": 100, "n_action_steps": 100, "dim_model": 512}
#
# chunk_size 100 -> 30 (5 s -> 1.5 s at 20fps). With per-step deltas the far end of a
# long chunk is close to unpredictable noise, and under L1 the model answers that by
# predicting ~0 there, spending capacity on steps it cannot know. A shorter horizon
# puts that capacity on the near future, which is the part that actually gets
# executed: microscope_inference.py runs n_action_steps=1 with temporal ensembling,
# so only element 0 is ever sent to the arm.
#
# Deliberately NOT changed yet, so this run stays interpretable:
#   vision_backbone     - resnet50+dilation is 25M params on ~50k frames (overfit risk),
#                         and it raises resolution, not the 1.8-sigma tip contrast that
#                         is the real perception limit.
#   precision-phase loss weighting - needs an ACT loss change; would confound this run.
POLICY_KWARGS: dict = {"chunk_size": 30, "n_action_steps": 30}

# --- Image augmentation ---
# Off by default upstream. Kept ON here, but note the justification is different from the
# 2026-09-10 run's: that one cited a MEASURED train/rollout brightness shift on the wrist
# cam, and this rig has no wrist cam - the dataset is a single microscope view.
#
# The reason it still earns its place is that the Basler runs ExposureAuto and GainAuto
# continuously against a fixed AutoTargetBrightness, so the absolute brightness of a frame
# depends on what is in the field of view at the time rather than being pinned. Between a
# recording session and a rollout the lamp, the objective or the dish can all move that.
# ColorJitter brightness (0.8, 1.2) hedges it. This is a hedge, not a correction of a
# measured shift - if a rollout ever shows one, measure it and revisit the range.
#
# AFFINE IS ZEROED ON PURPOSE. The upstream default includes RandomAffine with
# translate=(0.05, 0.05) at equal weight - up to 32px of image shift with the target
# action left unchanged. For a policy whose whole remaining job is resolving a ~50px
# tip-to-dot offset, that trains in exactly the invariance we need it NOT to have.
# Weight 0 drops a transform entirely (ImageTransforms skips weight <= 0).
USE_IMAGE_TRANSFORMS = True
GEOMETRIC_TRANSFORMS = ("affine",)  # zeroed: they move the image without moving the action

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

    _report_action_representation(meta)
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


def build_image_transforms() -> ImageTransformsConfig:
    """Upstream's transform set with the geometric ones removed.

    Photometric jitter teaches invariance to lighting, which is what we want. Geometric
    jitter teaches invariance to *position*, which for a visuomotor policy is the signal
    itself - the image moves and the supervised action does not, so the model learns that
    a shifted target needs no change of action.
    """
    cfg = ImageTransformsConfig(enable=USE_IMAGE_TRANSFORMS)
    for name in GEOMETRIC_TRANSFORMS:
        if name not in cfg.tfs:
            raise ValueError(
                f"GEOMETRIC_TRANSFORMS names {name!r}, which is not in the transform set "
                f"({sorted(cfg.tfs)}). Upstream must have renamed it - check before training, "
                f"because silently failing to zero it trains in a positional invariance."
            )
        cfg.tfs[name].weight = 0.0

    if USE_IMAGE_TRANSFORMS:
        kept = sorted(n for n, t in cfg.tfs.items() if t.weight > 0)
        print(f"[augment] enabled, up to {cfg.max_num_transforms} of {kept}")
        print(f"[augment] zeroed (geometric): {sorted(GEOMETRIC_TRANSFORMS)}")
    else:
        print("[augment] disabled")
    return cfg


def _report_action_representation(meta: LeRobotDatasetMetadata) -> None:
    """Check the dataset's action channel is the representation DELTA_ACTIONS claims.

    Absolute joint targets span the workspace (std ~25 deg); per-step deltas are
    ~0.3 deg and centred on zero. The two are trivially separable, and confusing them
    is the one mistake here with a hardware consequence, so it is worth failing early
    rather than discovering it when the arm hits a joint limit.
    """
    stats = meta.stats.get("action", {})
    names = list(meta.features.get("action", {}).get("names") or [])
    joints = [i for i, n in enumerate(names) if n.endswith(".pos")]
    if not joints or "std" not in stats:
        print("[actions] could not inspect the action channel; skipping the delta check.")
        return

    std = float(np.mean([stats["std"][i] for i in joints]))
    mean = float(np.mean([abs(stats["mean"][i]) for i in joints]))
    looks_delta = std < 5.0 and mean < 1.0
    print(
        f"[actions] joint action std={std:.3f} deg, |mean|={mean:.3f} deg "
        f"-> looks like {'DELTAS' if looks_delta else 'ABSOLUTE targets'}"
    )

    if looks_delta != DELTA_ACTIONS:
        raise SystemExit(
            f"\n  DELTA_ACTIONS={DELTA_ACTIONS} but {meta.repo_id!r} holds "
            f"{'per-step deltas' if looks_delta else 'absolute joint targets'} "
            f"(std={std:.3f} deg).\n"
            f"  Fix whichever is wrong:\n"
            f"    - to train on deltas:   NAME must point at a dataset built by\n"
            f"                            scripts/meca500/convert_to_delta_actions.py\n"
            f"    - to train on absolutes: set DELTA_ACTIONS = False\n"
        )

    if DELTA_ACTIONS:
        print("[actions] REMINDER: deploy this checkpoint with Meca500Config(relative_actions=True).")
        print("[actions] With it False the arm receives ~0.005 deg absolute targets and drives")
        print("[actions] into its lower joint limits.")


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
            image_transforms=build_image_transforms(),
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
