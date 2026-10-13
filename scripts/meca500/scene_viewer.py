"""Show a bench's collision model in the browser, to check it against the real setup.

Builds the scene in the robot's base frame and writes ONE self-contained HTML file (the
3D data is embedded, three.js comes from a CDN), then opens it in the default browser.
No 3D app needed - Windows' own 3D Viewer has been retired. Drag to orbit, right-drag to
pan, scroll to zoom; the checkboxes toggle each group.

Groups:
    obstacles   the bench's obstacle meshes (what the planner avoids)
    model       our robot (URDF collision meshes) + tool at ZERO joints
    cad robot   the robot exactly as exported from Fusion (if CAD_ROBOT_DIR is set)
                - should sit on top of "model"; anywhere they differ, the model is wrong
    home        our robot + tool at HOME_JOINTS
    markers     the optical axis (red line) and the tip at zero joints (red dot)
"""

import base64
import sys
import webbrowser
from pathlib import Path

import numpy as np
import trimesh
from rigs import require_home_joints

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.kinematics import LINK6_TO_FLANGE, load_stl

# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
# The Fusion export's robot links and where the robot's base frame sits in that CAD frame
# (see bench.yaml). None to leave the CAD robot out.
CAD_ROBOT_DIR = Path("cad/huanyu_setup/export")
CAD_BRF_ORIGIN = (-147.5, 85.5, 0.0)
OPTICAL_AXIS_XY = (237.5, -85.5)
OUT = Path("cad/huanyu_setup/scene_viewer.html")
# -----------------------------------------------------------------

REPO = Path(__file__).resolve().parents[2]

GROUPS = {  # name prefix -> (label, colour, opacity, visible by default)
    "obstacle": ("obstacles", "#9aa3ad", 0.85, True),
    "model": ("model (zero joints)", "#3c78e6", 0.55, True),
    "tool": ("model tool", "#3cc85a", 1.0, True),
    "cad": ("CAD robot", "#f08c28", 1.0, True),
    "home": ("home pose", "#a064dc", 0.45, False),
    "marker": ("markers", "#e61e1e", 1.0, True),
}


def build_scene() -> trimesh.Scene:
    bench = BenchConfig.load(RIG)
    kin = bench.kinematics()
    scene = trimesh.Scene()
    for i, rel in enumerate(bench.section("obstacles").get("meshes", [])):
        scene.add_geometry(trimesh.load_mesh(bench.root / rel), node_name=f"obstacle_{i}_{Path(rel).stem}")

    names = [kin.base_link] + [j.child for j in kin.joints]
    links = {n: load_stl(kin.link_meshes[n]) for n in names if n in kin.link_meshes}
    tool = (
        trimesh.load_mesh(bench.root / bench.section("tool")["mesh"])
        if bench.section("tool").get("mesh")
        else None
    )

    def add_arm(q, prefix):
        frames = kin.link_frames(q)
        for i, n in enumerate(names):
            if n in links:
                v, f = links[n]
                m = trimesh.Trimesh(v * 1000.0, f)
                m.apply_transform(frames[i])
                scene.add_geometry(m, node_name=f"{prefix}_{n}")
        if tool is not None:
            m = tool.copy()
            m.apply_transform(frames[-1] @ LINK6_TO_FLANGE)
            scene.add_geometry(m, node_name="tool_zero" if prefix == "model" else f"{prefix}_tool")

    add_arm(np.zeros(6), "model")
    add_arm(require_home_joints(RIG), "home")

    if CAD_ROBOT_DIR is not None:
        cad_dir = REPO / CAD_ROBOT_DIR
        for f in sorted(cad_dir.glob("meca500_link*.stl")) + sorted(cad_dir.glob("tool_*.stl")):
            m = trimesh.load_mesh(f)
            m.apply_translation(-np.asarray(CAD_BRF_ORIGIN))
            scene.add_geometry(m, node_name=f"cad_{f.stem}")

    x, y = OPTICAL_AXIS_XY
    scene.add_geometry(
        trimesh.creation.cylinder(radius=0.8, segment=[[x, y, -13], [x, y, 240]]), node_name="marker_axis"
    )
    tip = trimesh.creation.icosphere(radius=2.5)
    tip.apply_translation(kin.tip(np.zeros(6))[:3, 3])
    scene.add_geometry(tip, node_name="marker_tip")
    return scene


HTML = """<!doctype html><html><head><meta charset="utf-8"><title>__TITLE__</title>
<style>body{margin:0;overflow:hidden;font:13px system-ui,sans-serif}#ui{position:absolute;top:10px;left:10px;
background:#fffe;padding:10px 12px;border-radius:6px;box-shadow:0 1px 4px #0003}#ui label{display:block;margin:3px 0}
.sw{display:inline-block;width:10px;height:10px;margin:0 6px 0 2px;border-radius:2px}</style>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js",
"three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script></head><body>
<div id="ui"><b>__TITLE__</b><div style="color:#666;margin-bottom:6px">robot base frame, mm, z up</div></div>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
const GROUPS = __GROUPS__;
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setPixelRatio(devicePixelRatio);
renderer.setSize(innerWidth, innerHeight); document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#f4f5f7');
const cam = new THREE.PerspectiveCamera(40, innerWidth/innerHeight, 1, 10000); cam.up.set(0,0,1);
cam.position.set(-450,-650,550);
const ctl = new OrbitControls(cam, renderer.domElement); ctl.target.set(100,-60,120); ctl.update();
scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 2.2));
const d = new THREE.DirectionalLight(0xffffff, 1.5); d.position.set(300,-400,800); scene.add(d);
scene.add(new THREE.AxesHelper(100));
const groups = {};
for (const [p,[label,color,op,vis]] of Object.entries(GROUPS)) {
  const g = new THREE.Group(); g.visible = vis; scene.add(g);
  groups[p] = {g, mat: new THREE.MeshStandardMaterial({color, transparent: op<1, opacity: op,
    depthWrite: op>=1, side: THREE.DoubleSide, metalness: 0.05, roughness: 0.7})};
  const l = document.createElement('label');
  l.innerHTML = `<input type="checkbox" ${vis?'checked':''}><span class="sw" style="background:${color}"></span>${label}`;
  l.querySelector('input').onchange = e => g.visible = e.target.checked; document.getElementById('ui').appendChild(l);
}
const bytes = Uint8Array.from(atob('__GLB__'), c => c.charCodeAt(0));
new GLTFLoader().parse(bytes.buffer, '', gltf => {
  const meshes = []; gltf.scene.traverse(o => { if (o.isMesh) meshes.push(o); });
  for (const o of meshes) {
    const p = Object.keys(groups).find(k => o.name.startsWith(k) || (o.parent && o.parent.name.startsWith(k)));
    if (!p) continue;
    o.updateWorldMatrix(true, false); const m = o.clone(); m.matrix.copy(o.matrixWorld);
    m.matrix.decompose(m.position, m.quaternion, m.scale); m.material = groups[p].mat; groups[p].g.add(m);
  }
});
addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); renderer.setSize(innerWidth, innerHeight); });
(function loop(){ requestAnimationFrame(loop); renderer.render(scene, cam); })();
</script></body></html>"""


def main() -> None:
    import json

    scene = build_scene()
    glb = scene.export(file_type="glb")
    groups = {k: list(v) for k, v in GROUPS.items()}
    html = (
        HTML.replace("__TITLE__", f"{RIG} collision model")
        .replace("__GROUPS__", json.dumps(groups))
        .replace("__GLB__", base64.b64encode(glb).decode())
    )
    out = REPO / OUT
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding="utf-8")
    print(f"Wrote {out} ({out.stat().st_size // 1024} KB)")
    if "--no-open" not in sys.argv:
        webbrowser.open(out.as_uri())


if __name__ == "__main__":
    main()
