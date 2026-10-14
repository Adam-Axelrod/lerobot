"""Check our URDF kinematics against the robot's own pose readout. READ-ONLY.

The collision checker and the planner trust ``autocollect/kinematics.py``, which is built
from the URDF in mecademic_description/. That URDF came from a third party, so before any
planned path goes near the illuminator it must agree with the controller:

    predicted = inv(WRF) @ FK_flange(joints) @ TRF      vs      GetRtTargetCartPos()

The TARGET joints and pose are read, not the measured ones: monitor mode may not send commands,
and only the target values are streamed by default (GetRtJointPos would have to query the
controller). Samples are only taken while the arm is still, where the two are the same.

This connects in MONITOR mode (never moves the arm, never takes the control connection),
so run it in a second terminal while jogging with teleoperate_microscope.py. It samples
automatically whenever the arm has been still for a second at a pose clearly different
from the previous sample - just jog to ~10 varied poses (swing every joint) and pause at
each. Ctrl-C (or N_SAMPLES) prints the summary.

Pass: position error <= 0.5 mm and orientation error <= 0.1 deg at every pose. A constant
offset in z means the URDF base frame is not the BRF; a growing error means a link length.
"""

import logging
import sys
import time
from dataclasses import replace

import numpy as np
from rigs import rig_config

from lerobot.robots.meca500.autocollect.kinematics import (
    Meca500Kinematics,
    matrix_to_pose,
    pose_to_matrix,
    rotation_log,
)
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
N_SAMPLES = 10
STILL_S = 1.0  # how long the arm must be still before a sample is taken
STILL_TOL_DEG = 0.01
MIN_CHANGE_DEG = 10.0  # summed |joint change| since the last sample
POS_TOL_MM = 0.5
ROT_TOL_DEG = 0.1
# -----------------------------------------------------------------


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    robot = make_robot_from_config(replace(rig_config(RIG), cameras={}, monitor_mode=True))
    robot.connect()
    meca = robot.robot
    kin = Meca500Kinematics()
    trf = pose_to_matrix(meca.GetRtTrf())
    wrf = pose_to_matrix(meca.GetRtWrf())
    print(f"TRF (in FRF): {np.round(meca.GetRtTrf(), 3).tolist()}")
    print(f"WRF (in BRF): {np.round(meca.GetRtWrf(), 3).tolist()}")
    print(f"\nJog to {N_SAMPLES} varied poses and pause ~{STILL_S:.0f}s at each. Ctrl-C to finish early.\n")

    rows, last_sample, still_since, prev = [], None, None, None
    try:
        while len(rows) < N_SAMPLES:
            q = np.array(meca.GetRtTargetJointPos())
            now = time.monotonic()
            if prev is None or np.abs(q - prev).max() > STILL_TOL_DEG:
                still_since = now
            prev = q
            moved = last_sample is None or np.abs(q - last_sample).sum() > MIN_CHANGE_DEG
            if moved and now - still_since >= STILL_S:
                cart = pose_to_matrix(meca.GetRtTargetCartPos())
                pred = np.linalg.inv(wrf) @ kin.flange(q) @ trf
                dp = float(np.linalg.norm(pred[:3, 3] - cart[:3, 3]))
                dr = float(np.degrees(np.linalg.norm(rotation_log(pred[:3, :3] @ cart[:3, :3].T))))
                rows.append((q, dp, dr, pred[:3, 3] - cart[:3, 3]))
                last_sample = q
                flag = "ok" if dp <= POS_TOL_MM and dr <= ROT_TOL_DEG else "!!"
                print(
                    f"  {flag} #{len(rows):2d} joints {np.round(q, 1).tolist()}  "
                    f"pos err {dp:.3f} mm  rot err {dr:.4f} deg  (xyz {np.round(rows[-1][3], 3).tolist()})"
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.disconnect()

    if not rows:
        sys.exit("No samples taken.")
    dps, drs = np.array([r[1] for r in rows]), np.array([r[2] for r in rows])
    offsets = np.array([r[3] for r in rows])
    print(
        f"\n{len(rows)} poses: position error max {dps.max():.3f} mm (mean {dps.mean():.3f}), "
        f"orientation error max {drs.max():.4f} deg"
    )
    print(f"mean xyz offset (FK - robot): {np.round(offsets.mean(axis=0), 3).tolist()} mm")
    if dps.max() <= POS_TOL_MM and drs.max() <= ROT_TOL_DEG:
        print("PASS - the URDF kinematics match the controller.")
    else:
        print("FAIL - do not plan near obstacles until this is understood (see the module docstring).")
        print(f"example: FK pose {np.round(matrix_to_pose(kin.flange(rows[0][0])), 2).tolist()}")
        sys.exit(1)


if __name__ == "__main__":
    main()
