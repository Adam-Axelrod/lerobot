"""Check the bench's CAD model against reality by touching known points with the tip.

List 4-6 sharp, reachable features in bench.yaml under ``registration.points`` with their
CAD coordinates (e.g. a corner of the illuminator column, the underside of the lamp head,
an edge of the stage), then run this. For each point, in order:

    1. jog the tip (SpaceMouse; Enter toggles fine mode) until it just touches the feature,
    2. press SPACE in the status window to record it   (n = skip this point, q = finish).

It reports each point's error (measured - CAD) and the best rigid correction between the
two. Errors well inside safety.link_margin_mm mean the model is good and nothing needs to
change. If there is a consistent offset, it saves the correction to registration.json with
``apply: false``; set it to true after checking it makes sense, and the collision checker
will move every obstacle by it.

The tip position is our kinematics + tool.tip_pose_in_flange, the model the planner uses.
Owns the arm: close teleop/record first. Uses no camera.
"""

import json
import logging
import sys
from dataclasses import replace

import cv2
import numpy as np
from live_view import LiveView
from rigs import axis_signs, require_home_joints, rig_config

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.collision import fit_rigid
from lerobot.robots.meca500.autocollect.kinematics import rotation_log
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
GAIN_TR, GAIN_ROT = 20.0, 15.0
GAIN_TR_FINE, GAIN_ROT_FINE = 1.0, 1.0
# -----------------------------------------------------------------

WINDOW = "register_bench  (SPACE = record, n = skip, q = finish)"


def status_image(lines: list[str]) -> np.ndarray:
    img = np.zeros((40 + 24 * len(lines), 760, 3), np.uint8)
    for i, line in enumerate(lines):
        cv2.putText(
            img, line, (10, 28 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (230, 230, 230), 1, cv2.LINE_AA
        )
    return img


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    points = bench.section("registration").get("points") or []
    if len(points) < 3:
        sys.exit("bench.yaml needs >= 3 registration.points ({name, cad_xyz, description}).")
    kin = bench.kinematics()
    margin = float(bench.section("safety").get("link_margin_mm", 15.0))

    robot = make_robot_from_config(replace(rig_config(RIG), cameras={}, monitor_mode=True))
    teleop = make_teleoperator_from_config(
        Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
            gain_tr_fine=GAIN_TR_FINE,
            gain_rot_fine=GAIN_ROT_FINE,
            axis_signs=axis_signs(RIG),
            home_joints=require_home_joints(RIG),
        )
    )
    # Window first: if the display cannot open, nothing has connected to the arm yet.
    view = LiveView(WINDOW, figsize=(9, 3))
    robot.connect()
    teleop.connect()

    measured = {}
    try:
        for p in points:
            while True:
                teleop.get_action()
                obs = robot.get_observation()
                tip = kin.tip([obs[f"joint_{i + 1}.pos"] for i in range(6)])[:3, 3]
                cad = np.asarray(p["cad_xyz"], float)
                view.show(
                    status_image(
                        [
                            f"point: {p['name']}  -  {p.get('description', '')}",
                            f"CAD   {np.round(cad, 2).tolist()} mm",
                            f"tip   {np.round(tip, 2).tolist()} mm",
                            f"dist  {np.linalg.norm(tip - cad):.2f} mm",
                            "SPACE record    n skip    q finish",
                        ]
                    ),
                )
                if view.closed:
                    raise KeyboardInterrupt
                key = view.key()
                if key == " ":
                    measured[p["name"]] = (cad, tip.copy())
                    print(
                        f"  {p['name']:>20}: error {np.round(tip - cad, 2).tolist()} mm "
                        f"(|e| = {np.linalg.norm(tip - cad):.2f})"
                    )
                    break
                if key == "n":
                    break
                if key == "q":
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        view.close()
        teleop.disconnect()
        robot.disconnect()

    if not measured:
        sys.exit("Nothing recorded.")
    cad = np.array([v[0] for v in measured.values()])
    tip = np.array([v[1] for v in measured.values()])
    err = np.linalg.norm(tip - cad, axis=1)
    print(
        f"\n{len(err)} points: error max {err.max():.2f} mm, rms {np.sqrt(np.mean(err**2)):.2f} mm "
        f"(link margin {margin} mm)"
    )
    if len(err) < 3:
        return
    t = fit_rigid(cad, tip)
    resid = np.linalg.norm(cad @ t[:3, :3].T + t[:3, 3] - tip, axis=1)
    angle = np.degrees(np.linalg.norm(rotation_log(t[:3, :3])))
    print(
        f"best rigid correction: translate {np.round(t[:3, 3], 2).tolist()} mm, rotate {angle:.2f} deg; "
        f"residual after it {resid.max():.2f} mm max"
    )
    if err.max() < margin / 3:
        print("Model agrees with reality well inside the margin - no correction needed.")
        return
    bench.registration_path().write_text(
        json.dumps(
            {
                "apply": False,
                "t_robot_cad": t.tolist(),
                "points": {k: {"cad": v[0].tolist(), "measured": v[1].tolist()} for k, v in measured.items()},
            },
            indent=2,
        )
    )
    print(f"Saved {bench.registration_path()} with apply: false - review, then set apply: true to use it.")


if __name__ == "__main__":
    main()
