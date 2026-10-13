"""Turn a Fusion 360 export of a bench into the bench's collision model.

Export from Fusion with: Browser > right-click the TOP-LEVEL design > Save As Mesh >
STL (Binary), mm, Structure = One File Per Body, with the Meca500 at ZERO joints. Put the
files in CAD_DIR, then run this. It:

1. sorts the files by name - "Meca500_1_Link N" are robot links, any other part under
   "Meca500_1_" is the tool, everything else is an obstacle;
2. finds the robot base frame (BRF) in the CAD frame by fitting the URDF's links at zero
   joints to the CAD links (yaw + translation), and refuses if they do not overlay;
3. writes the obstacles in the BRF to assets/benches/<RIG>/obstacles/ and the tool in the
   flange frame (FRF) to assets/benches/<RIG>/tool.stl, deleting obstacle files that are
   no longer in the export;
4. finds the pipette tip (the thin, tapered end of the part named "Pipette") and writes
   obstacles.meshes and tool.tip_pose_in_flange into bench.yaml.

Re-run it whenever the CAD changes, then re-run suggest_touch_pose.py and plan_preview.py:
a new tool invalidates the touch pose.
"""

import json
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.kinematics import (
    LINK6_TO_FLANGE,
    Meca500Kinematics,
    load_stl,
    make_transform,
    matrix_to_pose,
    rot_z,
    sample_surface,
)

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
CAD_DIR = Path("cad/huanyu_setup/export")
ROBOT_KEY = "Meca500_1_"  # Fusion occurrence name of the robot, as it appears in file names
MAX_OVERLAY_MM = 2.5  # median URDF-to-CAD link distance above which the import refuses
# -----------------------------------------------------------------

REPO = Path(__file__).resolve().parents[2]


def part_name(path: Path) -> str:
    """'Huanyu's Setup_Huanyu's Setup_Meca500_1_Force Sensor_1_..._1_Force Sensor.stl' -> 'force_sensor'."""
    stem = path.stem
    if ROBOT_KEY in stem:
        stem = stem.split(ROBOT_KEY, 1)[1]
    else:
        stem = stem.split("_")[-3] if stem.count("_") >= 3 else stem
    stem = stem.split("_1_")[0]
    return re.sub(r"[^a-z0-9]+", "_", stem.lower()).strip("_")


def classify(files):
    links, tool, obstacles = {}, [], []
    for f in files:
        m = re.search(r"Link (\d)", f.stem)
        if ROBOT_KEY in f.stem and m:
            links[int(m.group(1))] = f
        elif ROBOT_KEY in f.stem:
            tool.append(f)
        else:
            obstacles.append(f)
    return links, tool, obstacles


def fit_base_frame(kin, link_files) -> tuple[np.ndarray, float]:
    """CAD -> BRF transform (yaw about z + translation) from URDF links at zero joints."""
    rng = np.random.default_rng(0)
    np.random.seed(0)  # trimesh's sample() draws from the global generator
    names = [kin.base_link] + [j.child for j in kin.joints]
    frames = kin.link_frames(np.zeros(6))
    urdf = []
    for i, n in enumerate(names):
        v, f = load_stl(kin.link_meshes[n])
        p = sample_surface(v * 1000.0, f, 3.0, rng)
        urdf.append(p @ frames[i][:3, :3].T + frames[i][:3, 3])
    urdf = np.vstack(urdf)
    cad = trimesh.util.concatenate([trimesh.load_mesh(f) for f in link_files.values()]).sample(150000)
    tree = cKDTree(cad)

    best = None
    for yaw in np.radians(np.arange(0, 360, 2)):
        r = rot_z(yaw)
        t = cad.mean(axis=0) - r @ urdf.mean(axis=0)
        for _ in range(8):  # translation-only ICP: URDF hulls sit inside the CAD shells
            w = urdf @ r.T + t
            _, idx = tree.query(w)
            t += np.median(cad[idx] - w, axis=0)
        d = np.median(tree.query(urdf @ r.T + t)[0])
        if best is None or d < best[0]:
            best = (d, r, t)
    d, r, t = best
    # The ICP above settles the yaw, but a median-distance fit is biased (the URDF's convex
    # collision hulls sit inside the CAD shells). Take the translation from geometry instead:
    # z = 0 is the base's bottom face, and link 1's outline is centred on the J1 axis in the
    # URDF and the CAD alike, so matching their bounding-box centres pins x/y.
    base_cad = trimesh.load_mesh(link_files[0])
    link1_cad = trimesh.load_mesh(link_files[1])
    v, f = load_stl(kin.link_meshes[names[1]])
    link1_urdf = v * 1000.0 @ frames[1][:3, :3].T + frames[1][:3, 3]
    c_urdf = r @ ((link1_urdf.min(axis=0) + link1_urdf.max(axis=0)) / 2)
    c_cad = (link1_cad.bounds[0] + link1_cad.bounds[1]) / 2
    t = np.array([c_cad[0] - c_urdf[0], c_cad[1] - c_urdf[1], base_cad.bounds[0][2]])
    d = float(np.median(tree.query(urdf @ r.T + t)[0]))
    t_cad_brf = make_transform(r, t)  # BRF points -> CAD
    return np.linalg.inv(t_cad_brf), float(d)


def pipette_tip(pipette: trimesh.Trimesh):
    """(tip point, unit axis from the thick end to the tip) of a tapered pipette mesh."""
    v = pipette.vertices
    c = v - v.mean(axis=0)
    u = np.linalg.eigh(c.T @ c)[1][:, -1]
    s = c @ u
    span = np.ptp(s)

    def radius(mask):
        seg = c[mask]
        return np.linalg.norm(seg - np.outer(seg @ u, u), axis=1).max()

    if radius(s > s.max() - 0.1 * span) > radius(s < s.min() + 0.1 * span):
        u, s = -u, -s  # make +u point at the thinner end
    tip = v[s > s.max() - 0.3].mean(axis=0)
    return tip, u


def main() -> None:
    bench = BenchConfig.load(RIG)
    kin = Meca500Kinematics()
    files = sorted((REPO / CAD_DIR).glob("*.stl"))
    links, tool_files, obstacle_files = classify(files)
    if len(links) != 7:
        sys.exit(f"Expected robot links 0-6 named '...{ROBOT_KEY}Link N...', found {sorted(links)}")
    print(
        f"robot links: {len(links)}   tool parts: {[part_name(f) for f in tool_files]}   "
        f"obstacles: {[part_name(f) for f in obstacle_files]}"
    )

    t_brf_cad, overlay = fit_base_frame(kin, links)
    yaw = np.degrees(np.arctan2(t_brf_cad[1, 0], t_brf_cad[0, 0]))
    origin = np.linalg.inv(t_brf_cad)[:3, 3]
    print(
        f"robot base frame in CAD: origin {np.round(origin, 2).tolist()} mm, yaw {yaw:.1f} deg; "
        f"URDF links overlay the CAD links to {overlay:.2f} mm (median)"
    )
    if overlay > MAX_OVERLAY_MM:
        sys.exit(f"Overlay {overlay:.2f} mm > {MAX_OVERLAY_MM} mm: is the CAD robot at ZERO joints?")

    # obstacles -> BRF
    out_dir = bench.root / "obstacles"
    out_dir.mkdir(exist_ok=True)
    written = []
    for f in obstacle_files:
        m = trimesh.load_mesh(f)
        m.apply_transform(t_brf_cad)
        name = f"{part_name(f)}.stl"
        m.export(out_dir / name)
        written.append(f"obstacles/{name}")
        print(
            f"  obstacle {name:28s} BRF bounds {np.round(m.bounds, 1).tolist()}  watertight {m.is_watertight}"
        )
    for stale in out_dir.glob("*.stl"):
        if f"obstacles/{stale.name}" not in written:
            stale.unlink()
            print(f"  removed stale {stale.name}")

    # tool -> FRF
    t_frf_brf = np.linalg.inv(kin.link_frames(np.zeros(6))[-1] @ LINK6_TO_FLANGE)
    t_frf_cad = t_frf_brf @ t_brf_cad
    parts = {part_name(f): trimesh.load_mesh(f) for f in tool_files}
    for m in parts.values():
        m.apply_transform(t_frf_cad)
    trimesh.util.concatenate(list(parts.values())).export(bench.root / "tool.stl")
    pip = next((m for n, m in parts.items() if "pipette" in n), None)
    if pip is None:
        sys.exit("No tool part named 'Pipette' - cannot locate the tip.")
    tip, axis = pipette_tip(pip)
    x = np.cross([0.0, 1.0, 0.0], axis)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross([1.0, 0.0, 0.0], axis)
    x /= np.linalg.norm(x)
    tip_pose = matrix_to_pose(make_transform(np.column_stack([x, np.cross(axis, x), axis]), tip))
    print(
        f"tool: {list(parts)} -> tool.stl; tip {np.round(tip, 2).tolist()} mm in the FRF "
        f"({np.linalg.norm(tip):.1f} mm from the flange), pipette {np.degrees(np.arccos(axis[2])):.1f} deg "
        f"off the flange axis"
    )

    # bench.yaml: replace the two generated lines, keep everything else
    yaml_path = bench.root / "bench.yaml"
    text = yaml_path.read_text(encoding="utf-8")
    text, n1 = re.subn(r"(?m)^  meshes: \[.*\]$", f"  meshes: [{', '.join(written)}]", text)
    text, n2 = re.subn(
        r"(?m)^  tip_pose_in_flange: .*$",
        f"  tip_pose_in_flange: {[round(float(v), 2) for v in tip_pose]}",
        text,
    )
    if n1 != 1 or n2 != 1:
        sys.exit("Could not find obstacles.meshes / tool.tip_pose_in_flange lines in bench.yaml to update.")
    yaml_path.write_text(text, encoding="utf-8")
    # Where the CAD sits relative to the robot, so scene_viewer.py can overlay the raw export.
    (bench.root / "cad_frame.json").write_text(
        json.dumps(
            {
                "cad_dir": CAD_DIR.as_posix(),
                "t_brf_cad": np.round(t_brf_cad, 6).tolist(),
                "overlay_mm": overlay,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"updated {yaml_path} - now re-run suggest_touch_pose.py and plan_preview.py")


if __name__ == "__main__":
    main()
