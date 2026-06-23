"""Record a SpaceMouse-teleop dataset on the Meca500 microscope/pipette rig.

Cameras: overhead (index 0) + microscope (index 2). Movement is scaled WAY down vs.
the default rig for fine pipette positioning. Produces datasets keyed
`overhead_cam` + `microscope_cam` — a distinct observation space from the wrist-cam rig.

Workflow: edit the CONFIG block below, then `python record_microscope.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Tip: find good exposure/focus first with scripts/meca500/check_camera.py
(set CAMERA_INDEX to 0 then 2), then paste the values below.
"""

import shutil
from pathlib import Path

from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig
from lerobot.scripts.lerobot_record import DatasetRecordConfig, RecordConfig, record
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
USER = "AdamAxelrod"
NAME = "microscope_pipette"
TASK = "move_pipette_under_microscope"

NUM_EPISODES = 100
FPS = 30
EPISODE_TIME_S = 60
RESET_TIME_S = 60
DISPLAY_DATA = True
PUSH_TO_HUB = True

# Movement scale (SpaceMouse at full deflection). Default rig is 50/30 — scaled down
# hard for fine pipette work. Drop to 1-2 mm/s for very fine positioning.
GAIN_TR = 5.0   # mm/s  translation
GAIN_ROT = 3.0  # deg/s rotation

# Camera tuning (driver-dependent units; tune with check_camera.py first).
# The microscope cam has no software focus (fixed lens ring) — focus it by hand.
OVERHEAD_EXPOSURE, OVERHEAD_FOCUS = -6, 100
MICROSCOPE_EXPOSURE = -6
# ------------------------------------------------------------------


def build_robot_config() -> Meca500MicroscopeConfig:
    """Microscope rig config with the CONFIG-block camera tuning applied.

    autoexposure/autofocus are already set in the defaults, so assigning
    exposure/focus after construction is safe (it skips the __post_init__ check).
    """
    cfg = Meca500MicroscopeConfig(id="meca500_microscope")
    cfg.cameras["overhead_cam"].exposure = OVERHEAD_EXPOSURE
    cfg.cameras["overhead_cam"].focus = OVERHEAD_FOCUS
    cfg.cameras["microscope_cam"].exposure = MICROSCOPE_EXPOSURE
    return cfg


def main() -> None:
    register_third_party_plugins()

    repo_id = f"{USER}/{NAME}"

    # Clear any stale local copy so re-runs don't trip the dataset sanity check.
    cache_dir = Path.home() / ".cache" / "huggingface" / "lerobot" / repo_id
    shutil.rmtree(cache_dir, ignore_errors=True)

    cfg = RecordConfig(
        robot=build_robot_config(),  # monitor_mode=True (default) — teleop owns activation
        dataset=DatasetRecordConfig(
            repo_id=repo_id,
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
        ),
        display_data=DISPLAY_DATA,
    )

    record(cfg)


if __name__ == "__main__":
    main()
