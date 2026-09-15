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

import contextlib
import sys

# HUB is unused by default but is part of the CONFIG vocabulary below: set
# CKPT = HUB to run the pushed Hub model instead of a local checkpoint.
from inference import HUB, resolve_policy  # noqa: F401

from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500.meca500 import Meca500
from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig
from lerobot.rollout.configs import EpisodicStrategyConfig, RolloutConfig
from lerobot.scripts.lerobot_rollout import rollout
from lerobot.teleoperators.meca500_home.config_meca500_home import Meca500HomeConfig
from lerobot.teleoperators.meca500_home.meca500_home import Meca500Home
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
RUN = "microscope_pipette_2026-09-10"
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
# The checkpoint has chunk_size=n_action_steps=100, so at 20fps full-chunk mode
# replans every 5s and executes the chunk open-loop in between. Each new chunk is
# computed from where the arm actually is by then, so its first action is a step
# change — which showed up as a burst of "Relative goal position ... clamped"
# warnings every 5s, decaying over the following few steps as the arm converged.
# Ensembling runs inference every step (n_action_steps=1 below) and exponentially
# averages overlapping chunk predictions, so there are no boundaries to jump.
# Cost: 20 forward passes/sec instead of 0.2 — watch for the loop dropping below
# FPS, and fall back to n_action_steps≈20 if so.
#
# THE SIGN MATTERS, and not in the intuitive direction. w_i = exp(-coeff * i)
# where w_0 weights the OLDEST contributing prediction (see ACTTemporalEnsembler
# in policies/act/modeling_act.py), so a POSITIVE coeff weights stale predictions
# MORE. At chunk_size=100/20fps the executed action's mean age is:
#
#   +0.01 (the ACT paper's value)  2.88s   freshest prediction gets 0.6% of weight
#    0.00 (uniform)                2.47s
#   -0.05                          0.94s   freshest gets 4.9%
#   -0.10                          0.48s   freshest gets 9.5%
#
# +0.01 is a ~3-second low-pass filter. That is why it cured the chunk-boundary
# wobble, and also why the arm never settled on the target: the policy's "I am on
# the dot, stop" prediction was being averaged with ~3s of "keep approaching".
# Measured against the demos, the rollout's fine phase barely decelerated (7.8°
# then 5.6° over the two halves of precision mode, vs the operator's 3.5° then
# 0.6°) and still moved 0.56° in its final 2s, where every demonstration was
# dead still. Negative keeps most of the smoothing while letting the current
# observation actually drive the action.
TEMPORAL_ENSEMBLE_COEFF: float | None = -0.05

# Per-step joint-delta clamps (degrees). None = unclamped.
#   MAX_REL_TARGET           applies when precision is off (coarse approach)
#   PRECISION_MAX_REL_TARGET applies when the policy predicts precision.state > 0.5
#
# PRECISION_MAX_REL_TARGET is set from what the operator actually demonstrated
# rather than guessed: in microscope_pipette_2026-09-10, per-step joint motion
# while precision.state was latched had a median of 0.0045 deg and a p95 of
# 0.056 deg. The previous 0.5 was ~10x looser than the largest fine-phase step in
# 100 demonstrations, so the clamp never once engaged — it was decoration. 0.06
# sits just above the demonstrated p95: coarse motion is untouched (it runs
# unclamped anyway), but once the policy asserts precision it can only creep at
# the speed a human creeps.
#
# This bounds the RATE of the drift, not its DIRECTION. If the policy is heading
# somewhere wrong it will still go there, just slowly enough to watch and abort.
MAX_REL_TARGET: float | None = None
PRECISION_MAX_REL_TARGET: float | None = 0.06

HOME_JOINTS = [70, 10, 10, 90, -80, 15]  # matches record_microscope.py, so rollouts start where demos did
# Drive the arm to HOME_JOINTS before episode 0. The rollout homes BETWEEN episodes
# but not before the first one (see home_arm), so without this the first rollout
# starts from wherever the arm was left.
HOME_BEFORE_START = True
HOME_TIMEOUT_S = 30.0
NUM_EPISODES = 10
# The rollout runs for a fixed time — the policy cannot end an episode itself — so
# this must exceed the longest demonstration or a run gets cut off mid-approach.
# The 2026-09-10 dataset (100 episodes) ranges 15.7-43.9s (median 23.7s, p95 31.6s),
# so 60s leaves headroom for the policy being slower than the operator, plus time to
# sit at the endpoint. Note this dataset has a much longer tail than 2026-09-02
# (which topped out at 32.5s): 45s would clear the median comfortably but cut off the
# slowest demonstrated approaches.
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
    robot = Meca500(Meca500Config(id="meca500", monitor_mode=False, cameras={}))
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
