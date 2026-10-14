"""The Meca500 rigs, by registered name - the single list every script selects from.

Two benches share this checkout and differ only in their cameras (and in which laptop
Ethernet port the arm is cabled to, which is host configuration, not repo configuration):

    meca500                    two UC60 webcams, no microscope. The original rig.
    meca500_microscope         bench 1: UC60 overhead + wrist + the VMS700 microscope cam.
                               Everything up to microscope_pipette_2026-09-10 is this rig.
    meca500_basler             bench 2 today: the Basler microscope cam and nothing else.
    meca500_microscope_basler  bench 2 once its two UC60 arm cameras arrive.

Scripts import RIGS from here rather than each keeping their own mapping, so adding a rig
is one edit and no script can be left knowing about a rig the others do not.

A rig is a *robot config class*, not a folder of scripts: what changes between benches is
the camera set, while record/teleop/inference are identical. Forking the scripts per bench
instead would mean fixing every future bug twice.
"""

from __future__ import annotations

from lerobot.robots.meca500.config_meca500 import Meca500Config
from lerobot.robots.meca500_microscope.config_meca500_basler import Meca500BaslerConfig
from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig
from lerobot.robots.meca500_microscope.config_meca500_microscope_basler import (
    Meca500MicroscopeBaslerConfig,
)

# Module-level newline, so the multi-line SystemExit below reads as a list of lines
# rather than one string full of escapes.
NL = chr(10)

RIGS = {
    "meca500": Meca500Config,
    "meca500_microscope": Meca500MicroscopeConfig,
    "meca500_basler": Meca500BaslerConfig,
    "meca500_microscope_basler": Meca500MicroscopeBaslerConfig,
}


# The auto-home pose each bench returns to between episodes, per rig.
#
# These are NOT interchangeable, and getting it wrong is the expensive kind of mistake:
# record drives here between EVERY episode, so a pose measured on another bench sends the
# arm somewhere the sample, the objective or the pipette holder may now be. reset.py has
# warned about this since before there was a second bench; the difference now is that there
# IS one, so the copies in each script were an accident waiting to happen. One definition
# per rig, here.
#
# None means "not measured on this bench yet" and is deliberate - it fails loudly rather
# than silently inheriting the other bench's pose, which is exactly the failure to avoid.
HOME_JOINTS: dict[str, list[float] | None] = {
    "meca500": [0.0, 0.0, 0.0, 0.0, 90.0, 0.0],
    "meca500_microscope": [70.0, 10.0, 10.0, 90.0, -80.0, 15.0],
    "meca500_basler": [-90.0, 40.0, 20.0, 90.0, 80.0, -110.0],
    "meca500_microscope_basler": None,
}


# SpaceMouse axis signs per rig, as [vx, vy, vz, wx, wy, wz].
#
# The teleop drives the arm with MoveLinVelWrf, i.e. in the Meca's WORLD (base) frame, so
# how the base is physically bolted down decides which way a SpaceMouse push moves the tool.
# That is a property of the bench, not of the SpaceMouse, which is why it lives here.
#
# Bench 2 has the Meca500 base turned 180 degrees about the vertical relative to bench 1.
# That maps x -> -x, y -> -y, z unchanged; angular velocities are axial vectors and follow
# the same rotation, so wx and wy flip too and wz does not. Bench 1's signs with indices
# 0, 1, 3 and 4 negated gives the entry below.
#
# NOTE this only works because the turn is 180 degrees. Sign flips can reverse an axis but
# never SWAP two, so a base rotated 90 degrees could not be corrected here at all - it would
# need the x and y channels exchanged. Same trap as the camera orientation.
#
# These affect operator feel only. The recorded action and observation are joint angles, so
# a dataset stays schema-compatible across benches regardless of what is set here.
AXIS_SIGNS: dict[str, list[float]] = {
    "meca500": [-1.0, -1.0, 1.0, 1.0, -1.0, -1.0],
    "meca500_microscope": [-1.0, -1.0, 1.0, 1.0, -1.0, -1.0],
    # Base turned 180 degrees about the vertical - see above.
    "meca500_basler": [1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
    "meca500_microscope_basler": [1.0, 1.0, 1.0, -1.0, 1.0, -1.0],
}


# Bota force/torque sensor on each bench (configs in src/lerobot/robots/meca500/bota/).
# The benches carry different models, and the bota_driver needs the matching JSON:
#   original setup (bench 1)  MiniONE      - BFT-MIPS-ENET-CG on COM5
#   Huanyu's setup (bench 2)  MiniONE Pro  - BFT-MIPS-SER-CG  on COM8
BOTA_SENSORS: dict[str, str] = {
    "meca500": "miniONE",
    "meca500_microscope": "miniONE",
    "meca500_basler": "miniONE_pro",
    "meca500_microscope_basler": "miniONE_pro",
}


def bota_config_path(rig: str):
    """The bota_driver JSON for the force sensor fitted on ``rig``'s bench."""
    from lerobot.robots.meca500.bota import BOTA_CONFIGS

    if rig not in BOTA_SENSORS:
        raise ValueError(f"Unknown rig {rig!r}. Available: {sorted(BOTA_SENSORS)}")
    return BOTA_CONFIGS[BOTA_SENSORS[rig]]


def axis_signs(rig: str) -> list[float]:
    """SpaceMouse axis signs for ``rig``, as [vx, vy, vz, wx, wy, wz]."""
    if rig not in AXIS_SIGNS:
        raise ValueError(f"Unknown rig {rig!r}. Available: {sorted(AXIS_SIGNS)}")
    return AXIS_SIGNS[rig]


def home_joints(rig: str) -> list[float] | None:
    """The auto-home pose for ``rig``, or None if it has not been measured yet."""
    if rig not in HOME_JOINTS:
        raise ValueError(f"Unknown rig {rig!r}. Available: {sorted(HOME_JOINTS)}")
    return HOME_JOINTS[rig]


def require_home_joints(rig: str) -> list[float]:
    """The auto-home pose for ``rig``, refusing to fall back to another bench's."""
    home = home_joints(rig)
    if home is None:
        raise SystemExit(
            NL.join(
                [
                    "",
                    f"No HOME_JOINTS measured for rig {rig!r} yet.",
                    "",
                    "Recording drives the arm to this pose between every episode, so it has to",
                    "be a pose on THIS bench - another rig's would send the arm into a",
                    "different desk.",
                    "",
                    "To set it:",
                    "  1. python scripts/meca500/teleoperate_microscope.py  (jog to a start pose)",
                    "  2. python scripts/meca500/show_joints.py             (read the six values)",
                    f"  3. paste them into HOME_JOINTS[{rig!r}] in scripts/meca500/rigs.py",
                    "",
                ]
            )
        )
    return home


def rig_config(rig: str, **overrides):
    """Build the robot config for ``rig``, with ``id`` defaulting to the rig name.

    Raises a ValueError naming the valid rigs rather than a KeyError, since this is
    driven by a hand-edited constant at the top of each script.
    """
    if rig not in RIGS:
        raise ValueError(f"Unknown rig {rig!r}. Available: {sorted(RIGS)}")
    overrides.setdefault("id", rig)
    return RIGS[rig](**overrides)
