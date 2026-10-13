"""Dry-run the planner for a bench - no robot, no camera.

Plans N_EPISODES with the real bench model and reports what recording would do: how often
samples are rejected (and why), episode lengths, per-frame joint steps against the human
demos, and the tightest clearance to any obstacle. Then writes a 3D scene,
``assets/benches/<RIG>/preview.html`` (opens in the browser; preview.glb beside it for Blender/Fusion) with the obstacles, the arm at the start and touch
pose of the first few episodes and every planned tip path.
Look at it before letting the arm move.

Until the scope is calibrated, SYNTHETIC_CALIBRATION stands in a fake field of view
around the nominal touch point, so the geometry can be previewed early. Recording refuses
to use it.
"""

import logging
import sys
from pathlib import Path

import numpy as np

from lerobot.robots.meca500.autocollect.bench import BenchConfig, PlannerLimits
from lerobot.robots.meca500.autocollect.calibration import ScopeCalibration
from lerobot.robots.meca500.autocollect.kinematics import LINK6_TO_FLANGE, load_stl
from lerobot.robots.meca500.autocollect.planner import Planner, PlanningError

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
N_EPISODES = 100
SEED0 = 0
SHOW_ARMS = 3  # draw the arm at start/touch for this many episodes
# Used only if the bench has no scope_calibration.json yet.
SYNTHETIC_CALIBRATION = True
SYNTHETIC_UM_PER_PX = 3.0
SYNTHETIC_IMAGE_SIZE = (427, 640)
# -----------------------------------------------------------------


def synthetic_calibration(kin, limits) -> ScopeCalibration:
    touch = kin.tip(limits.touch_nominal_joints)[:3, 3]
    s = SYNTHETIC_UM_PER_PX / 1000
    w, h = SYNTHETIC_IMAGE_SIZE
    px = np.array([[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]], float)
    xy = (px - [w / 2, h / 2]) * s + touch[:2]
    return ScopeCalibration.fit(px, xy, touch[2], SYNTHETIC_IMAGE_SIZE)


def export_scene(bench, kin, plans, path: Path) -> None:
    import trimesh

    scene = trimesh.Scene()
    for p in bench.section("obstacles").get("meshes", []):
        m = trimesh.load_mesh(bench.root / p)
        m.visual.face_colors = [150, 150, 150, 200]
        scene.add_geometry(m, node_name=f"obstacle_{Path(p).stem}")
    for b in bench.section("obstacles").get("boxes", []):
        m = trimesh.creation.box(extents=b["size"])
        m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(b.get("yaw_deg", 0)), [0, 0, 1]))
        m.apply_translation(b["center"])
        m.visual.face_colors = [150, 150, 150, 200]
        scene.add_geometry(m, node_name="obstacle_" + b.get("name", "box"))
    for c in bench.section("obstacles").get("cylinders", []):
        m = trimesh.creation.cylinder(radius=c["radius"], height=c["height"])
        m.apply_translation(np.asarray(c["base"]) + [0, 0, c["height"] / 2])
        m.visual.face_colors = [150, 150, 150, 200]
        scene.add_geometry(m, node_name="obstacle_" + c.get("name", "cylinder"))

    link_names = [kin.base_link] + [j.child for j in kin.joints]
    link_meshes = {}
    for name in link_names:
        if name in kin.link_meshes:
            v, f = load_stl(kin.link_meshes[name])
            link_meshes[name] = trimesh.Trimesh(v * 1000.0, f, process=True)
    tool_rel = bench.section("tool").get("mesh")
    tool_mesh = trimesh.load_mesh(bench.root / tool_rel) if tool_rel else None

    def add_arm(q, color, tag):
        frames = kin.link_frames(q)
        for i, name in enumerate(link_names):
            if name in link_meshes:
                m = link_meshes[name].copy()
                m.apply_transform(frames[i])
                m.visual.face_colors = color
                scene.add_geometry(m, node_name=f"{tag}_{name}")
        if tool_mesh is not None:
            m = tool_mesh.copy()
            m.apply_transform(frames[-1] @ LINK6_TO_FLANGE)
            m.visual.face_colors = color
            scene.add_geometry(m, node_name=f"{tag}_tool")

    for k, plan in enumerate(plans):
        tips = np.array([kin.tip(q)[:3, 3] for q in plan.joints[::2]])
        tube = trimesh.load_path(tips)
        scene.add_geometry(tube, node_name=f"path_{k}")
        if k < SHOW_ARMS:
            add_arm(plan.joints[0], [60, 120, 220, 160], f"start{k}")
            add_arm(plan.joints[-1], [60, 200, 90, 160], f"touch{k}")
    scene.export(path)
    return scene


def main() -> None:
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()
    limits = PlannerLimits.from_bench(bench)
    try:
        calib = bench.calibration()
        print(f"Scope calibration: {calib.residual_px_rms:.2f} px rms, {calib.mm_per_px * 1000:.2f} um/px")
    except FileNotFoundError:
        if not SYNTHETIC_CALIBRATION:
            raise
        calib = synthetic_calibration(kin, limits)
        print(
            f"No scope calibration yet - using a SYNTHETIC {SYNTHETIC_UM_PER_PX} um/px field of view "
            "around the nominal touch point."
        )
    checker = bench.collision_checker(kin)
    planner = Planner(kin, checker, calib, limits)

    plans, rejections, failures = [], {}, 0
    for i in range(N_EPISODES):
        try:
            plans.append(planner.sample_episode(SEED0 + i))
        except PlanningError:
            failures += 1
        for k, v in planner.rejections.items():
            rejections[k] = rejections.get(k, 0) + v
        print(f"\r  planned {i + 1}/{N_EPISODES}", end="", flush=True)
    print()
    if not plans:
        sys.exit(f"No feasible episodes. Rejections: {rejections}")

    attempts = len(plans) + sum(rejections.values())
    dur = np.array([p.duration_s for p in plans])
    clear = np.array([p.clearance.min_mm for p in plans])
    coarse, precision = [], []
    for p in plans:
        steps = np.abs(np.diff(p.joints, axis=0)).max(axis=1)
        flag = p.precision[1:] == 1
        coarse.append(steps[~flag])
        precision.append(steps[flag])
    coarse, precision = np.concatenate(coarse), np.concatenate(precision)

    print(
        f"\n{len(plans)} plans, {failures} seeds infeasible; acceptance {len(plans) / attempts:.0%} of samples"
    )
    print(f"rejections: {dict(sorted(rejections.items(), key=lambda kv: -kv[1]))}")
    print(f"duration s     min {dur.min():.1f}  median {np.median(dur):.1f}  max {dur.max():.1f}")
    print(
        f"coarse step    p95 {np.percentile(coarse, 95):.3f}  max {coarse.max():.3f} deg/frame  (demos: 0.53 / 1.34)"
    )
    print(
        f"precision step p95 {np.percentile(precision, 95):.4f}  max {precision.max():.4f} deg/frame  (demos: 0.038 / 0.107)"
    )
    worst = plans[int(np.argmin(clear))]
    print(
        f"clearance      min {clear.min():.1f} mm beyond the margin "
        f"({worst.clearance.where} vs {worst.clearance.obstacle}, seed {worst.seed}); median {np.median(clear):.1f}"
    )

    from scene_viewer import obstacle_groups, write_html

    scene = export_scene(bench, kin, plans, bench.root / "preview.glb")
    groups = {
        **obstacle_groups(bench),
        "obstacle": ("other obstacles", "#9aa3ad", 0.85, True),
        "path": ("planned tip paths", "#e61e1e", 0.8, True),
        "start": (f"arm at start (first {SHOW_ARMS})", "#3c78e6", 0.45, True),
        "touch": (f"arm at touch (first {SHOW_ARMS})", "#3cc85a", 0.6, True),
    }
    write_html(scene, bench.root / "preview.html", groups, f"{RIG}: {len(plans)} planned episodes")


if __name__ == "__main__":
    main()
