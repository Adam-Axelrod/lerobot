"""SpaceMouse teleop for the Meca500 microscope/pipette rig.

Jog the stripper pipette under the microscope with a 3DConnexion SpaceMouse. The
overhead camera is on index 0 and the microscope camera on index 2. Movement is
scaled WAY down vs. the default rig (50 mm/s / 30 deg/s) for fine positioning.

Workflow: edit the CONFIG block below, then `python teleoperate_microscope.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Tip: find good exposure/focus first with scripts/meca500/check_camera.py
(set CAMERA_INDEX to 0 then 2), then paste the values below.

Equivalent to:
    lerobot-teleoperate --robot.type=meca500_microscope \
        --teleop.type=meca500_spacemouse --teleop.gain_tr=5 --teleop.gain_rot=3 \
        --display_data=true
"""

import sys

from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
DISPLAY_DATA = True
FPS = 60

# Movement scale (SpaceMouse at full deflection). Default rig is 50/30 — scaled down
# hard for fine pipette work. Drop to 1-2 mm/s for very fine positioning.
GAIN_TR = 50.0   # mm/s  translation
GAIN_ROT = 30.0  # deg/s rotation

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

    cfg = TeleoperateConfig(
        robot=build_robot_config(),  # monitor_mode=True (default) — teleop owns activation
        teleop=Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
        ),
        fps=FPS,
        display_data=DISPLAY_DATA,
    )

    try:
        teleoperate(cfg)
    except DeviceNotConnectedError as e:
        sys.exit(f"\nERROR: {e}")


if __name__ == "__main__":
    main()
