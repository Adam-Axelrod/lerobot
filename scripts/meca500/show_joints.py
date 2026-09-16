"""Print the Meca500's joint angles, live, in a form you can paste into rigs.py.

Use it to capture a bench's auto-home pose: jog the arm to where each episode should
start (teleoperate_microscope.py), then read the six values off here and paste them into
HOME_JOINTS in scripts/meca500/rigs.py.

READ-ONLY and safe to leave running. It connects in monitor mode, so it never activates
the arm, never sends a motion command, and never takes the control connection - which
means you can run it in a SECOND terminal alongside teleop and watch the numbers move as
you jog. Monitoring is a separate port from control, so the two do not fight.

    python scripts/meca500/show_joints.py                # follow the arm live
    python scripts/meca500/show_joints.py --once         # print one sample and exit
    python scripts/meca500/show_joints.py --rig meca500  # a different rig's connection

Ctrl-C to stop.
"""

import argparse
import logging
import sys
import time
from dataclasses import replace

from rigs import RIGS, rig_config

from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins


def joint_values(observation: dict) -> list[float]:
    """The six joint angles, in joint order, from a robot observation."""
    return [v for k, v in observation.items() if k.startswith("joint_") and k.endswith(".pos")]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rig", choices=sorted(RIGS), default="meca500_basler")
    parser.add_argument("--once", action="store_true", help="print a single sample and exit")
    parser.add_argument("--hz", type=float, default=5.0, help="refresh rate (default 5)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()

    # cameras={} skips all camera startup - this tool only wants joints, and opening the
    # Basler here would pointlessly take it away from a preview running next door.
    # monitor_mode=True is what keeps this read-only.
    cfg = replace(rig_config(args.rig), cameras={}, monitor_mode=True)
    robot = make_robot_from_config(cfg)
    robot.connect()

    try:
        print(f"\nReading {args.rig} joints (monitor mode - the arm is not being driven).")
        print("Paste the line below into HOME_JOINTS in scripts/meca500/rigs.py.\n")
        while True:
            joints = joint_values(robot.get_observation())
            formatted = ", ".join(f"{j:.1f}" for j in joints)
            if args.once:
                print(f"    {args.rig!r}: [{formatted}],")
                break
            # \r keeps it on one line so the numbers tick in place while you jog.
            print(f"\r    [{formatted}]    ", end="", flush=True)
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        joints = joint_values(robot.get_observation())
        formatted = ", ".join(f"{j:.1f}" for j in joints)
        print(f"\n\nFinal pose - paste into scripts/meca500/rigs.py:\n\n    {args.rig!r}: [{formatted}],\n")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    try:
        main()
    except (ConnectionError, TimeoutError) as e:
        sys.exit(f"\nERROR: {e}")
