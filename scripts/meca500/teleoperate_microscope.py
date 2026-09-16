"""SpaceMouse teleop for a Meca500 microscope/pipette rig.

Jog the stripper pipette under the microscope with a 3DConnexion SpaceMouse. Works on
either bench - set RIG in the CONFIG block below, which owns the camera set.
Drive coarsely (gain_tr/gain_rot),
then press **Enter** to toggle precision mode — the gains drop to gain_tr_fine/
gain_rot_fine for fine positioning (press Enter again to toggle back to coarse).
Press **H** to reset: the arm homes to HOME_JOINTS and the precision toggle clears,
the same MoveJoints reset that record_microscope.py runs between episodes. No
recording here; use it to practice the toggle/reset and tune gains before running
record_microscope.py.

Workflow: edit the CONFIG block below, then `python teleoperate_microscope.py`.
Ctrl-C to stop, edit, up-arrow, run again.

Tip: confirm camera identities and tune exposure/focus first with
scripts/meca500/camera_tool.py (it saves them; nothing to paste here).

Equivalent to (with RIG as --robot.type):
    lerobot-teleoperate --robot.type=meca500_basler \
        --teleop.type=meca500_spacemouse --teleop.gain_tr=50 --teleop.gain_rot=30 \
        --teleop.gain_tr_fine=5 --teleop.gain_rot_fine=3 --display_data=true
"""

import sys

from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig, teleoperate
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_plugins
from rigs import axis_signs
from rigs import home_joints as rig_home_joints
from rigs import rig_config

# ----------------------------- CONFIG -----------------------------
# Which bench this run is on - the ONLY line to change when the laptop moves between
# setups. The rig determines the camera set; see scripts/meca500/rigs.py for the list.
#   "meca500_microscope"  bench 1: UC60 overhead + wrist + the VMS700 microscope cam
#   "meca500_basler"      bench 2: the Basler microscope cam only (no arm cams yet)
RIG = "meca500_basler"

DISPLAY_DATA = True
FPS = 20

# Coarse movement scale (SpaceMouse at full deflection); the default until Enter toggles.
GAIN_TR = 50.0  # mm/s  translation
GAIN_ROT = 30.0  # deg/s rotation

# Fine (precision) scale, used while the Enter toggle is on under the microscope.
# Drop to 1-2 mm/s for very fine positioning.
GAIN_TR_FINE = 5.0  # mm/s  translation
GAIN_ROT_FINE = 3.0  # deg/s rotation

# Reset ("home") pose for **H**: the arm returns here and the precision toggle drops,
# the same MoveJoints reset record runs between episodes. It lives per-rig in rigs.py.
#
# If this bench has no pose recorded yet, H falls back to wherever the arm is standing when
# teleop starts - so it stays useful (and safe) while you are jogging around hunting for the
# pose to put in rigs.py. Unlike record, teleop does not refuse: jogging is exactly how you
# find the pose in the first place, so refusing here would be a deadlock.
HOME_TIMEOUT_S = 30.0

# Camera exposure/focus is NOT configured here - the UC60/VMS700 cams read
# src/lerobot/robots/meca500/camera_settings.json (tune with camera_tool.py), and the
# Basler keeps exposure/gain in its own stored user set (tune in the pylon Viewer).
# Which cameras exist at all is the rig's business - see RIG above.
# ------------------------------------------------------------------


def read_current_joints() -> list[float]:
    """The arm's present joint angles, read without activating or moving it."""
    from dataclasses import replace

    from lerobot.robots.utils import make_robot_from_config

    robot = make_robot_from_config(replace(rig_config(RIG), cameras={}, monitor_mode=True))
    robot.connect()
    try:
        obs = robot.get_observation()
        return [v for k, v in obs.items() if k.startswith("joint_") and k.endswith(".pos")]
    finally:
        robot.disconnect()


def main() -> None:
    register_third_party_plugins()

    home = rig_home_joints(RIG)
    if home is None:
        home = read_current_joints()
        print(f"No HOME_JOINTS set for rig {RIG!r} in rigs.py - H returns the arm to where it is now.")
        print(f"    {RIG!r}: {[round(j, 1) for j in home]},   <- paste into rigs.py")

    cfg = TeleoperateConfig(
        # monitor_mode=True (default) — teleop owns activation. Camera indices are
        # resolved by USB identity and exposure/focus loaded from camera_settings.json,
        # both inside __post_init__.
        robot=rig_config(RIG),
        teleop=Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
            gain_tr_fine=GAIN_TR_FINE,
            gain_rot_fine=GAIN_ROT_FINE,
            axis_signs=axis_signs(RIG),
            home_joints=home,
            home_timeout_s=HOME_TIMEOUT_S,
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
