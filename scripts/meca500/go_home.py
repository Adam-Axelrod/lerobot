"""Bring the arm home along a collision-checked path, wherever a stopped session left it.

    python scripts/meca500/go_home.py

record_auto.go_home: if the tip is low it first backs out along the pipette, then a checked joint
path home. Prints where the arm was (tip height, clearance) first. Refuses rather than guesses if
no collision-free route exists.
"""

import os

import numpy as np
import record_auto as ra
from eval_auto import deaf
from rigs import require_home_joints, rig_config

from lerobot.processor import make_default_processors
from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_scripted.config_meca500_scripted import Meca500ScriptedConfig
from lerobot.teleoperators.meca500_scripted.meca500_scripted import Meca500Scripted
from lerobot.utils.import_utils import register_third_party_plugins

RIG = "meca500_basler"


def main() -> None:
    register_third_party_plugins()
    bench = BenchConfig.load(RIG)
    planner = ra.build_planner(bench)
    stage, model, hmap = ra.connect_stage(bench, planner.calib)
    stager = ra.StagePlanner(planner, model, hmap, planner.calib)
    robot = make_robot_from_config(
        rig_config(
            RIG,
            monitor_mode=False,
            relative_actions=False,
            max_relative_target=ra.MAX_REL_TARGET,
            precision_max_relative_target=ra.PRECISION_MAX_REL_TARGET,
        )
    )
    teleop = Meca500Scripted(Meca500ScriptedConfig(id="meca500_scripted"))
    home = np.asarray(require_home_joints(RIG), dtype=float)
    robot.connect()
    robot.configure()
    teleop.connect()
    try:
        ra.make_ready(robot.robot)
        stager.place(stage.position())
        q = np.asarray(robot.robot.GetRtJointPos(), dtype=float)
        tip = planner.kin.tip(q)[:3, 3]
        print(
            f"arm at {np.round(q, 2).tolist()}, tip {tip[2] - planner.calib.surface_z_mm:.2f} mm above the dish, "
            f"clearance {planner.checker.clearance(q).min_mm:.1f} mm"
        )
        ra.go_home(robot, teleop, planner, home, deaf(), make_default_processors())
        print(f"home: {np.abs(np.asarray(robot.robot.GetRtJointPos()) - home).max():.3f} deg off")
    finally:
        for dev in (robot, teleop, stage):
            try:
                dev.disconnect()
            except Exception:
                pass
    os._exit(0)


if __name__ == "__main__":
    main()
