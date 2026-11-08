"""After the robot has been slid on the table: measure the slide and re-register the obstacle scene.

    python scripts/meca500/apply_slide.py --before <scope_calibration backup from before the slide>
    python scripts/meca500/apply_slide.py --reset      # back at the original position
    python scripts/meca500/apply_slide.py --replace --before <the pinned original>   # 2 -> 4 mm

The microscope, the lamp, the stage and the dish stay put while the robot base moves by d, so in
robot coordinates every one of them moves by -d. A full auto_calibrate.py run after the slide
re-measures the scope calibration (and the dish height map); this compares it with the one from
before the slide. The robot-frame point under the image centre moved by -d, and the change in the
calibration's rotation is the slide's rotation about the vertical. Then it writes
registration.json (BenchConfig.registration_transform, apply: true), which moves every obstacle -
CAD meshes and primitives - by the measured slide, and slide_measurement.json with the numbers for
the paper. The policy is told nothing.

It also PINS the pre-slide in-plane calibration (autocollect/slide.py): a copy of the --before
calibration, whose pixel <-> robot XY map every method keeps - otherwise scripted, scripted_asr
and the expert would simply be recalibrated and the slide would test nothing. The dish HEIGHT is
not pinned: every method takes it from the current calibration and dish map, kept up to date by
the routine height check, as at 0 mm. eval_auto refuses to run if the slide files disagree.

The height change it reports is the slide's own (base tilt) plus the dish's drift since --before
was measured. Run the height check just before sliding, so that --before is minutes old and the
change is mostly the slide; slide_measurement.json records how old --before was.

The obstacles are moved by the translation only, exact at the microscope where the arm works
closest to them. The rotation is reported but NOT registered: it comes from the change in the
calibration's heading, and full calibrations with the robot never moved scatter by about 1 deg
(-1.09 to +0.03 deg, 2026-10-14..11-04), because the field is only ~5 mm across. Registering it
would turn that noise into 2-4 mm out at the robot base. MAX_ROTATION_DEG only refuses a rotation
too large to be noise (2026-11-08: a clamped slide by hand gave -0.60 deg).
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np

from lerobot.robots.meca500.autocollect.bench import BenchConfig
from lerobot.robots.meca500.autocollect.calibration import ScopeCalibration
from lerobot.robots.meca500.autocollect.slide import MEASUREMENT, PINNED_CALIBRATION

RIG = "meca500_basler"
MAX_ROTATION_DEG = 1.5


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--before", type=Path, help="the scope calibration from before the slide (a .bak.json)")
    ap.add_argument("--reset", action="store_true", help="remove the slide registration (robot back in place)")
    ap.add_argument(
        "--replace", action="store_true",
        help="a further slide (e.g. 2 -> 4 mm) without returning to 0 mm: --before must be the ORIGINAL "
        "calibration already pinned; the slide is measured from it and the registration replaced",
    )
    args = ap.parse_args()
    bench = BenchConfig.load(RIG)
    reg_path, out_path = bench.registration_path(), bench.root / MEASUREMENT
    pinned_cal = bench.root / PINNED_CALIBRATION
    if args.reset:
        for p in (reg_path, out_path, pinned_cal):
            if p.exists():
                p.rename(p.with_name(f"{p.stem}.{time.strftime('%Y%m%d-%H%M%S')}.bak.json"))
        print("Slide registration and pinned calibration removed: the scene is back where the CAD puts it.")
        return
    if args.before is None:
        ap.error("give --before, or --reset")
    if reg_path.exists() and not args.replace:
        sys.exit(f"{reg_path} exists: run --reset first, or --replace with the ORIGINAL calibration as --before.")
    if args.replace:
        # The registration is absolute (CAD -> robot = -d, d measured from the original calibration),
        # so a further slide is measured and registered against the true 0 mm reference only if
        # --before is that original - the file pinned at the first slide.
        if not pinned_cal.exists():
            sys.exit("--replace needs a slide in force (no pinned calibration): use a plain run.")
        if json.loads(Path(args.before).read_text(encoding="utf-8")) != json.loads(pinned_cal.read_text(encoding="utf-8")):
            sys.exit(f"--before {args.before} is not the pinned original calibration {pinned_cal.name}: refusing.")

    before = ScopeCalibration.load(args.before)
    after = bench.calibration()
    w, h = after.image_size
    centre = np.array([(w - 1) / 2, (h - 1) / 2])
    d = before.pixel_to_xy(centre)[0] - after.pixel_to_xy(centre)[0]  # robot moved +d, the scene -d

    def heading(cal) -> float:
        ex = cal.pixel_to_xy(centre + [1.0, 0.0])[0] - cal.pixel_to_xy(centre)[0]
        return float(np.degrees(np.arctan2(ex[1], ex[0])))

    rot = (heading(after) - heading(before) + 180) % 360 - 180
    dz = after.surface_z_mm - before.surface_z_mm
    age_min = (bench.calibration_path().stat().st_mtime - args.before.stat().st_mtime) / 60
    print(f"Measured slide of the robot base: {d[0]:+.3f}, {d[1]:+.3f} mm (|d| {np.linalg.norm(d):.3f} mm)")
    print(
        f"rotation about the vertical {rot:+.3f} deg; dish height in the robot frame changed {dz * 1000:+.0f} um "
        f"(slide + {age_min:.0f} min of drift between the two calibrations)"
    )
    if abs(rot) > MAX_ROTATION_DEG:
        sys.exit(f"Rotation {rot:.2f} deg > {MAX_ROTATION_DEG}: re-seat the robot squarely and recalibrate.")

    shutil.copy(args.before, pinned_cal)
    t = np.eye(4)
    t[:2, 3] = -d
    reg_path.write_text(
        json.dumps({"t_robot_cad": t.tolist(), "apply": True, "source": "apply_slide.py"}, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    out_path.write_text(
        json.dumps(
            {
                "slide_mm": [round(float(v), 4) for v in d],
                "slide_norm_mm": round(float(np.linalg.norm(d)), 4),
                "rotation_deg": round(rot, 4),
                "height_change_after_slide_um": round(dz * 1000, 1),
                "minutes_between_calibrations": round(age_min, 1),
                "before": str(args.before),
                "after": str(bench.calibration_path()),
                "methods_use": f"{PINNED_CALIBRATION} for pixel <-> XY; the current calibration and dish map for height",
                "measured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(f"Wrote {reg_path} (obstacles moved by {-d[0]:+.3f}, {-d[1]:+.3f} mm) and {out_path}.")
    print(f"Pinned for the methods (pixel <-> XY only): {args.before.name} -> {PINNED_CALIBRATION}.")


if __name__ == "__main__":
    main()
