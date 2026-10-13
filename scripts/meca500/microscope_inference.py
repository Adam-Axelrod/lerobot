"""Run policy inference on the Meca500 microscope/pipette rig.

The general inference script for the microscope rig (overhead + wrist + microscope
cameras) and the 7-dim action space (6 joints + the predicted `precision.state`).
It runs any trained policy on the arm — whether you're validating an intermediate
training checkpoint or deploying the final model for real use. CKPT selects which:
the final local checkpoint (the default), a specific earlier step to compare
against, or the pushed Hub model. Checkpoint resolution is shared with
inference.py so both scripts behave identically; see `resolve_policy` there.

Rollouts from different checkpoints get distinct dataset repo_ids
(rollout_<RUN>_<CKPT>), so evaluating a second checkpoint cannot silently
overwrite the first.

When the policy's predicted `precision.state` crosses 0.5, Meca500.send_action clamps
per-step joint motion to PRECISION_MAX_REL_TARGET instead of MAX_REL_TARGET, so the
arm "scales down its own movements" under the microscope. Tune both (values are
joint-space degrees per step).

Action representation
---------------------
RELATIVE_ACTIONS must match the dataset the checkpoint was trained on: a policy
trained on scripts/meca500/convert_to_delta_actions.py output emits per-step joint
DELTAS, which the robot integrates onto its current target pose. A mismatch is
checked at startup (check_action_representation) and refuses to run, because
driving a delta policy in absolute mode commands ~0.005 deg joint angles on the
first step. Note that home_arm() deliberately stays absolute - the homing teleop
speaks joint angles, not deltas.

Workflow: edit the CONFIG block below, then `python microscope_inference.py`.
Ctrl-C to stop, edit CKPT (or anything else), up-arrow, run again.

Equivalent to:
    lerobot-rollout --strategy.type=episodic \
        --robot.type=<RIG> --robot.monitor_mode=False \
        --robot.relative_actions=<RELATIVE_ACTIONS> \
        --robot.max_relative_target=<MAX_REL_TARGET> \
        --robot.precision_max_relative_target=<PRECISION_MAX_REL_TARGET> \
        --teleop.type=meca500_home --teleop.home=<HOME_JOINTS for RIG> \
        --policy.path=outputs/train/<RUN>/checkpoints/<CKPT>/pretrained_model \
        --dataset.repo_id=<USER>/rollout_<RUN>_<CKPT> \
        --dataset.single_task=<TASK> --dataset.push_to_hub=False
"""

import contextlib
import sys

# HUB is unused by default but is part of the CONFIG vocabulary below: set
# CKPT = HUB to run the pushed Hub model instead of a local checkpoint.
from inference import HUB, resolve_policy  # noqa: F401
from rigs import require_home_joints, rig_config

from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.rollout.configs import EpisodicStrategyConfig, RolloutConfig
from lerobot.scripts.lerobot_rollout import rollout
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.teleoperators.meca500_home.meca500_home import Meca500Home
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"

# Which bench this rollout runs on. The rig owns the camera set, and it MUST match the
# rig the checkpoint was trained against.
#
# THIS RUN NEEDS BENCH 1, NOT THE BASLER BENCH. microscope_pipette_2026-09-10 takes
# THREE visual inputs - overhead_cam, wrist_cam and microscope_cam (the VMS700) - so it
# cannot load against a microscope-only rig at all: make_policy fails on the input
# features before anything touches the arm. Running it means moving the robot's network
# cable to bench 1 and having both UC60 webcams and the VMS700 connected.
RIG = "meca500_microscope"
RUN = "microscope_pipette_2026-09-10"

# This checkpoint PREDATES the delta conversion - it was trained on absolute joint
# targets, so the robot must pass them through unchanged. check_action_representation()
# reads the checkpoint's own training dataset at startup and refuses to run on a
# mismatch, so this pairing cannot silently go wrong in either direction.
RELATIVE_ACTIONS = False
# Which policy to run (see resolve_policy in inference.py):
#   None       → the FINAL local checkpoint under outputs/train/<RUN>/checkpoints/
#   "025000"   → that specific local checkpoint, to compare against an earlier step
#   HUB        → the pushed Hub model <USER>/<RUN>_model
CKPT: str | None = None
TASK = "move_pipette_under_microscope"

# ACT inference mode:
#   None    → full-chunk inference (fast, wobbles at chunk boundaries)
#   float   → temporal ensembling (smooth, every-step inference)
#
# Ensembling runs inference every step (n_action_steps=1 below) and exponentially
# averages overlapping chunk predictions, so there are no chunk boundaries to jump
# at. Cost: 20 forward passes/sec instead of one per chunk — watch the loop rate.
# This checkpoint has chunk_size=100 (5s at 20fps), so each pass is dearer than the
# chunk_size=30 models; the 2026-09-10 run measured 19.7Hz against a 20Hz target.
#
# THE SIGN MATTERS, and not in the intuitive direction. w_i = exp(-coeff * i) where
# w_0 weights the OLDEST contributing prediction (see ACTTemporalEnsembler in
# policies/act/modeling_act.py), so a POSITIVE coeff weights stale predictions MORE.
# At chunk_size=100 the executed action's mean age is:
#
#   +0.01   2.88s   (freshest prediction gets 0.6% of the weight)
#   -0.05   0.94s   (freshest gets 4.9%)
#   -0.10   0.48s   (freshest gets 9.5%)
#
# +0.01 here, which is the OPPOSITE of the right answer for every delta model, and
# deliberately so. This checkpoint was trained on absolute actions that - because the
# SpaceMouse teleop records a position readback rather than a command - were identical
# to observation.state in 97.4% of frames. So every prediction it ensembles is an
# estimate of "where the arm already is", and a recency-weighted average of those
# commands no motion: at -0.05 this model crawled at 0.010 deg/step with straightness
# 0.04 (undirected jitter) and never arrived. It only moves at all because a
# ~3-second-stale forecast differs from the present pose.
#
# That is to say: this model is driven by prediction lag, not by anything it learned.
# +0.01 is the setting under which it does the most, not a setting that is correct.
# Do not carry this value over to a delta checkpoint.
TEMPORAL_ENSEMBLE_COEFF: float | None = 0.01

# Per-step joint-delta clamps (degrees). None = unclamped.
#   MAX_REL_TARGET           applies when precision is off (coarse approach)
#   PRECISION_MAX_REL_TARGET applies when the policy predicts precision.state > 0.5
#
# In absolute mode the clamp bounds |goal - present|, which is the per-step motion -
# the same quantity a delta clamp bounds, so the numbers are measured the same way.
# RE-MEASURE THESE PER DATASET; they are not portable between benches. Per-step joint
# motion over the 100 episodes of microscope_pipette_2026-09-10:
#
#              p50      p95      p99      max
#   coarse     0.5352   1.5985   2.2433   4.5757
#   precision  0.0045   0.0562   0.0805   0.1805
#
# Note these are ~3x larger than the Basler bench's (coarse max 4.58 vs 1.34), which
# is exactly why the values below differ from the ones this file carried for the
# basler run - the old bench simply moves further per step.
#
# The rule is "allow anything a human did, block anything they did not": 5.0 sits just
# above the largest coarse step demonstrated here, 0.2 just above the largest fine one.
# So neither clamp touches demonstrated behaviour, and both catch a runaway. If the
# pipette is fragile enough that you would rather lose the top of the demonstrated
# range than risk a lunge, 0.06 (the fine p95) is the conservative setting — it will
# engage regularly and slow convergence.
#
# This bounds the RATE of an error, not its DIRECTION. A policy heading somewhere
# wrong still goes there, just slowly enough to watch and abort.
MAX_REL_TARGET: float | None = 5.0
PRECISION_MAX_REL_TARGET: float | None = 0.2

# From rigs.py, so rollouts start exactly where the demos did. Taking it from the rig
# rather than copying it is what stops this file drifting from record_microscope.py -
# a rollout homing to the other bench's pose would start every episode off-distribution.
HOME_JOINTS = require_home_joints(RIG)
# Drive the arm to HOME_JOINTS before episode 0. The rollout homes BETWEEN episodes
# but not before the first one (see home_arm), so without this the first rollout
# starts from wherever the arm was left.
HOME_BEFORE_START = True
HOME_TIMEOUT_S = 30.0
NUM_EPISODES = 10
# The rollout runs for a fixed time — the policy cannot end an episode itself — so
# this must exceed the longest demonstration or a run gets cut off mid-approach.
# microscope_pipette_2026-09-10 (100 episodes) ranges 15.7-43.9s (median 23.7s, p95
# 31.6s), so 60s is ~1.4x the longest demonstration. This dataset has a much longer
# tail than the Basler one, which is why it needs 60s where that needed 45.
EPISODE_TIME_S = 60
RESET_TIME_S = 10
FPS = 20
DISPLAY_DATA = True
# Spoken phase announcements ("Recording episode 3", "Reset the environment").
# False = console log only; the same messages are still written to the terminal.
PLAY_SOUNDS = False
# ------------------------------------------------------------------


def home_arm() -> None:
    """Drive the arm to HOME_JOINTS before the rollout starts.

    EpisodicStrategy.run() goes straight into its episode loop -- it homes between
    episodes (the meca500_home teleop drives the reset phase) but never before
    episode 0. Every demonstration in the training set started at HOME_JOINTS, so
    without this the first rollout begins from an out-of-distribution pose, and
    with MAX_REL_TARGET=None coarse motion is unclamped while the policy works out
    what to do from a state it has never seen.

    Mirrors reset.py's sequence, including ``cameras={}``: homing needs no camera,
    and the rig's three streams share one USB bus, so opening them here just to
    close them again before the rollout re-opens them is worth avoiding.
    """
    # relative_actions stays False here no matter what RELATIVE_ACTIONS says, and that
    # is not an oversight: Meca500Home.get_action() returns ABSOLUTE joint angles. Fed
    # to a robot in relative mode they would be ADDED to the current pose - homing to
    # [70, 10, 10, 90, -80, 15] from anywhere near there would command roughly double
    # it, straight past the joint limits. Only the policy speaks deltas.
    robot = make_robot_from_config(
        rig_config(RIG, monitor_mode=False, cameras={}, relative_actions=False)
    )
    teleop = Meca500Home(Meca500HomeConfig(id="meca500_home", home=HOME_JOINTS))

    robot.connect()  # activates + homes to factory zero
    robot.configure()
    teleop.connect()
    try:
        print(f"[home] driving to {HOME_JOINTS} before the first episode")
        robot.send_action(teleop.get_action())
        # Block until the motion queue drains, or the teardown below ClearMotion()s
        # the move mid-flight and the rollout starts from a half-completed pose.
        robot.robot.WaitIdle(timeout=HOME_TIMEOUT_S)
        print("[home] reached")
    finally:
        teleop.disconnect()
        # Release the control socket WITHOUT deactivating. Meca500.disconnect()
        # calls DeactivateRobot(), which drops the brakes: the arm goes limp and
        # sags out of the pose we just set, and the rollout's own connect() then
        # re-runs ActivateAndHome on top. Leaving it activated means the rollout
        # reconnects to an arm already holding HOME_JOINTS, and its
        # ActivateAndHome is a fast no-op on an already-homed robot.
        #
        # This is still a graceful mecademicpy Disconnect(), not a dropped socket
        # -- an ungraceful drop faults the controller with "control connection
        # dropped". Falls back to the full disconnect if anything goes wrong, so
        # the arm is never left both energised and unreachable.
        try:
            robot.robot.Disconnect()
            robot.robot.WaitDisconnected(timeout=5)
            robot._connected = False
            print("[home] control socket released, arm still holding position")
        except Exception as e:
            print(f"[home] gentle disconnect failed ({e}); falling back to full disconnect")
            with contextlib.suppress(Exception):
                robot.disconnect()


def check_action_representation(policy_path) -> None:
    """Fail if RELATIVE_ACTIONS disagrees with the data the checkpoint was trained on.

    Reads the dataset the checkpoint records in its train_config.json and inspects its
    action statistics: absolute joint targets span the workspace (std ~25 deg), per-step
    deltas are ~0.3 deg and centred near zero. Getting this pair wrong is the one
    mistake here with an immediate hardware consequence - a delta policy driven in
    absolute mode commands ~0.005 deg joint angles, i.e. "go to zero", on the first
    step - so it is checked before anything energises the arm.

    Never blocks the run on its own uncertainty: if the dataset is not available
    locally it warns and continues, rather than making a missing cache fatal.
    """
    import json

    import numpy as np

    from lerobot.datasets.dataset_metadata import LeRobotDatasetMetadata

    try:
        repo_id = json.loads((policy_path / "train_config.json").read_text())["dataset"]["repo_id"]
        meta = LeRobotDatasetMetadata(repo_id)
        stats, names = meta.stats["action"], list(meta.features["action"]["names"])
        joints = [i for i, n in enumerate(names) if n.endswith(".pos")]
        std = float(np.mean([stats["std"][i] for i in joints]))
        mean = float(np.mean([abs(stats["mean"][i]) for i in joints]))
    except Exception as e:
        print(f"[actions] could not verify the action representation ({e}); trusting RELATIVE_ACTIONS.")
        return

    looks_delta = std < 5.0 and mean < 1.0
    print(
        f"[actions] {repo_id} -> joint action std={std:.3f} deg, |mean|={mean:.3f} deg "
        f"-> {'DELTAS' if looks_delta else 'ABSOLUTE targets'}; RELATIVE_ACTIONS={RELATIVE_ACTIONS}"
    )
    if looks_delta != RELATIVE_ACTIONS:
        raise SystemExit(
            f"\n  REFUSING TO RUN: the checkpoint was trained on "
            f"{'per-step deltas' if looks_delta else 'absolute joint targets'} but "
            f"RELATIVE_ACTIONS={RELATIVE_ACTIONS}.\n"
            f"  Set RELATIVE_ACTIONS = {looks_delta} (or point RUN at a checkpoint that matches).\n"
            f"  Running as configured would command "
            f"{'~0.005 deg absolute joint angles on the first step' if looks_delta else 'joint deltas of tens of degrees'}.\n"
        )


def main() -> None:
    register_third_party_plugins()

    label, policy_path = resolve_policy(USER, RUN, CKPT)
    check_action_representation(policy_path)

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
        robot=rig_config(
            RIG,
            monitor_mode=False,
            relative_actions=RELATIVE_ACTIONS,
            max_relative_target=MAX_REL_TARGET,
            precision_max_relative_target=PRECISION_MAX_REL_TARGET,
        ),
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
        play_sounds=PLAY_SOUNDS,
    )

    # After the config is built (so a bad CKPT or config fails before the arm
    # moves) and before the rollout takes over the control connection.
    if HOME_BEFORE_START:
        home_arm()

    rollout(cfg)


if __name__ == "__main__":
    main()
