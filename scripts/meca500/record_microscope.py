"""Record a SpaceMouse-teleop dataset on a Meca500 microscope/pipette rig.

Works on either bench - set RIG in the CONFIG block below. The rig owns the camera set,
so nothing else here changes when the laptop moves between setups. The operator drives
coarsely (gain_tr/gain_rot) until the pipette is under the microscope, then presses
**Enter** to toggle precision mode — the SpaceMouse gains drop to gain_tr_fine/
gain_rot_fine for fine positioning (press Enter again to toggle back to coarse). The
toggle state (`precision.state`) is recorded as an action channel and reset to coarse
on the between-episode auto-home.

Uses the auto-home flow (record.record_with_reset) so the arm homes to
HOME_JOINTS before the first episode and after each demo, and the precision toggle
resets for the next approach. Every episode therefore starts from the same pose.

Workflow: edit the CONFIG block below, then `python record_microscope.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Tip: confirm camera identities and tune exposure/focus first with
scripts/meca500/camera_tool.py (it saves them; nothing to paste here).
"""

from record import record_with_reset
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
# Which bench this run is on - the ONLY line to change when the laptop moves between
# setups. The rig determines the camera set; see scripts/meca500/rigs.py for the list.
#   "meca500_microscope"  bench 1: UC60 overhead + wrist + the VMS700 microscope cam
#   "meca500_basler"      bench 2: the Basler microscope cam only (no arm cams yet)
RIG = "meca500_basler"

USER = "AdamAxelrod"

# One dataset per rig, keyed off RIG so switching bench cannot append episodes recorded
# with one camera set onto a dataset recorded with another. That mistake is unrecoverable
# after the fact - the frames are already interleaved and the shapes may not even differ -
# so the name is derived rather than being a second constant to remember to edit.
DATASETS = {
    "meca500_microscope": "microscope_pipette_2026-09-10",
    "meca500_basler": "basler_pipette_2026-09-16",
}
NAME = DATASETS[RIG]
TASK = "move_pipette_under_microscope"

NUM_EPISODES = 100
# On meca500_basler the Basler caps the achievable rate: it needs binning=2 (set in the rig
# config) to clear 5.8 fps at all, and then cannot exceed 1/exposure. 20 fps therefore needs
# the camera's exposure under ~45 ms - it was auto-exposing to 90.9 ms (i.e. 11 fps) when
# this was measured. Check with `python scripts/meca500/basler_probe.py --serial 40585204`
# and buy the headroom with more illumination or gain, saved into the camera's user set.
FPS = 20
EPISODE_TIME_S = 120
RESET_TIME_S = 60
DISPLAY_DATA = True
# Spoken phase announcements ("Recording episode 3", "Reset the environment").
# False = console log only; the same messages are still written to the terminal.
PLAY_SOUNDS = False
PUSH_TO_HUB = True

# RESUME=True appends new episodes to the existing dataset at repo_id (USER/NAME),
# both locally and on the Hub — use this to add to a previous recording.
# RESUME=False creates a fresh dataset; with CLEAR_EXISTING_CACHE=True it first
# deletes the local cache for repo_id (and a later push overwrites the Hub repo).
RESUME = False
CLEAR_EXISTING_CACHE = True

# Coarse movement scale (SpaceMouse at full deflection); the default until Enter toggles.
GAIN_TR = 50.0  # mm/s  translation
GAIN_ROT = 30.0  # deg/s rotation

# Fine (precision) scale, used while the Enter toggle is on under the microscope.
# Drop to 1-2 mm/s for very fine positioning.
GAIN_TR_FINE = 5.0  # mm/s  translation
GAIN_ROT_FINE = 3.0  # deg/s rotation

# Auto-home target (executed between episodes; also resets the precision toggle).
# Lives per-rig in rigs.py, NOT here: it is a property of the bench, and a pose copied
# from the other one would drive the arm into a different desk between every episode.
HOME_TIMEOUT_S = 30.0

# Camera exposure/focus is NOT configured here - the UC60/VMS700 cams read
# src/lerobot/robots/meca500/camera_settings.json (tune with camera_tool.py), and the
# Basler keeps exposure/gain in its own stored user set (tune in the pylon Viewer).
# Which cameras exist at all is the rig's business - see RIG above.
# ------------------------------------------------------------------


def main() -> None:
    register_third_party_plugins()

    # Refuses (with instructions) if this bench's home pose has not been measured yet,
    # rather than silently inheriting the other bench's.
    home_joints = require_home_joints(RIG)

    cfg = RecordConfig(
        # monitor_mode=True (default) — teleop owns activation. Camera indices are
        # resolved by USB identity and exposure/focus loaded from camera_settings.json,
        # both inside __post_init__.
        robot=rig_config(RIG),
        dataset=DatasetRecordConfig(
            repo_id=f"{USER}/{NAME}",
            single_task=TASK,
            num_episodes=NUM_EPISODES,
            fps=FPS,
            episode_time_s=EPISODE_TIME_S,
            reset_time_s=RESET_TIME_S,
            push_to_hub=PUSH_TO_HUB,
        ),
        teleop=Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
            gain_tr_fine=GAIN_TR_FINE,
            gain_rot_fine=GAIN_ROT_FINE,
            axis_signs=axis_signs(RIG),
            home_joints=home_joints,
            home_timeout_s=HOME_TIMEOUT_S,
        ),
        display_data=DISPLAY_DATA,
        play_sounds=PLAY_SOUNDS,
        resume=RESUME,
    )

    record_with_reset(cfg, clear_existing_cache=CLEAR_EXISTING_CACHE)


if __name__ == "__main__":
    main()
