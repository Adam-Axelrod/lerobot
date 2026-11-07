"""Pre-flight for an eval_auto.py session: GO / NO-GO, read-only, the arm does not move.

    python scripts/meca500/preflight.py

Reads eval_auto.py's CONFIG block and checks, in order:
    config     METHOD and the settings that must go with it (no image shift, a software slide is
               only noted, whole chunks for the *_asr methods)
    slide      the physical-slide state is consistent (autocollect/slide.py) and which calibration the
               methods will use - pinned pre-slide, or the normal one
    files      scope calibration residual, dish height map present and agreeing with it, MODEL
               present and trained on a delta dataset, ASR site file and perception package
    focus      the focus K-Cube reads the calibrated position (needs Kinesis closed)
    robot      connects in MONITOR mode (cannot move), no error state, joints read, arm at home
    camera     a frame of the right size, lit
    detector   Huanyu's tip detector loads and runs on the live feed; with the arm at home, no
               detection at or above WATCH_MIN_CONFIDENCE in N_FRAMES frames
Close Kinesis, teleop and the web portal first. Exit code 0 on GO.
"""

import json
import os
import sys
import tempfile
import time
import traceback
from dataclasses import replace
from pathlib import Path

import numpy as np

N_FRAMES = 10
HOME_TOL_DEG = 0.5

results: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    results.append((name, bool(ok), detail))
    print(f"[{'ok' if ok else 'NO'}] {name}{': ' + detail if detail else ''}")
    return bool(ok)


def main() -> None:
    os.chdir(Path(__file__).resolve().parents[2])  # eval_auto's paths are relative to the repo root
    import eval_auto as ev
    import record_auto
    from rigs import require_home_joints, rig_config

    from lerobot.robots.meca500.autocollect.bench import BenchConfig
    from lerobot.robots.meca500.autocollect.dish_map import DishHeightMap
    from lerobot.robots.meca500.autocollect.focus_check import check_focus
    from lerobot.robots.meca500.autocollect.slide import split_benches
    from lerobot.robots.meca500.autocollect.stage import StageModel
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.utils.import_utils import register_third_party_plugins

    register_third_party_plugins()
    asr = ev.METHOD in ("policy_asr", "scripted_asr")
    learned = ev.METHOD in ("policy", "policy_asr")
    print(f"eval_auto: METHOD={ev.METHOD} NUM_EPISODES={ev.NUM_EPISODES} FIRST_EPISODE={ev.FIRST_EPISODE} "
          f"SEED_BASE={ev.SEED_BASE} SEARCH={ev.SEARCH}\n")

    # config
    check("config: METHOD known", ev.METHOD in ("policy", "policy_asr", "scripted", "scripted_asr", "expert", "human"), ev.METHOD)
    check("config: IMAGE_SHIFT_PX 0", ev.IMAGE_SHIFT_PX == 0 or not asr, str(ev.IMAGE_SHIFT_PX))
    if any(ev.BASE_OFFSET_MM):
        print(f"[note] SOFTWARE SLIDE {tuple(ev.BASE_OFFSET_MM)} mm: every method aims through the shifted map; rows labelled software_slide")
    if asr:
        check("config: whole chunks", ev.COARSE_ACTION_STEPS is None and ev.TEMPORAL_ENSEMBLE_COEFF is None)

    # slide state
    bench0 = BenchConfig.load(ev.RIG)
    try:
        benches = split_benches(bench0)
        m = benches.measurement
        check(
            "slide state consistent", True,
            "original position, normal calibration" if m is None else
            f"PHYSICAL SLIDE {m.get('slide_mm')} mm in force: methods use the pinned pre-slide calibration",
        )
        bench = benches.method
    except RuntimeError as e:
        check("slide state consistent", False, str(e))
        bench = bench0

    # files
    try:
        calib = bench.calibration()
        check("scope calibration", calib.residual_px_rms <= record_auto.MAX_CALIBRATION_RESIDUAL_PX,
              f"{bench.calibration_path().name}, residual {calib.residual_px_rms:.2f} px")
        hmap = DishHeightMap.load(bench.dish_map_path())
        model = StageModel.from_bench(bench.section("scope"), calib)
        dz = abs(hmap.surface_z(model.reference_xy) - calib.surface_z_mm)
        check("dish height map agrees with the calibration", dz <= record_auto.MAX_MAP_MISMATCH_MM,
              f"{bench.dish_map_path().name}, {dz * 1000:.0f} um apart")
    except Exception as e:
        check("calibration files", False, repr(e))
    if learned:
        p = Path(ev.MODEL)
        ok = p.exists() and (p / "train_config.json").exists()
        detail = ev.MODEL
        if ok:
            ds = json.loads((p / "train_config.json").read_text(encoding="utf-8"))["dataset"]["repo_id"]
            ok, detail = ds.endswith("_delta"), f"{ev.MODEL} (trained on {ds})"
        check("MODEL present, delta-trained", ok, detail)
    runtime = None
    if asr:
        site = Path(ev.ASR_SITE_YAML)
        if check("ASR site file", site.exists(), str(site)):
            import yaml

            onnx = Path(yaml.safe_load(site.read_text())["inference"]["onnx_package_path"])
            check("ASR perception package", onnx.exists(), str(onnx))
        try:
            if not site.is_file():
                raise FileNotFoundError(site)  # load_config would fall back to ASR's defaults
            from action_state_runtime.config.runtime import load_config
            from action_state_runtime.runtime import ActionStateRuntime

            t = time.perf_counter()
            runtime = ActionStateRuntime(config=load_config(str(site)), backend_mode="real", audit_dir=tempfile.mkdtemp())
            runtime.preload_detector()
            check("ASR detector loads", True, f"{time.perf_counter() - t:.0f} s")
        except Exception as e:
            check("ASR detector loads", False, repr(e))

    # focus
    try:
        check("focus K-Cube at the calibrated position", True, f"{check_focus(bench.section('scope')):.4f} mm")
    except Exception as e:
        check("focus K-Cube at the calibrated position", False, str(e))

    # robot + camera + detector
    robot = None
    try:
        robot = make_robot_from_config(replace(rig_config(ev.RIG), monitor_mode=True))
        robot.connect()
        check("robot connects (monitor mode: cannot move)", True)
        err = robot.robot.GetStatusRobot().error_status
        check("robot not in error", not err)
        obs = robot.get_observation()
        q = np.array([obs[f"joint_{j + 1}.pos"] for j in range(6)], float)
        home = np.asarray(require_home_joints(ev.RIG), float)
        at_home = np.abs(q - home).max() <= HOME_TOL_DEG
        check("arm at home", at_home, f"max joint offset {np.abs(q - home).max():.2f} deg")
        frame = obs[bench.camera_key]
        w, h = calib.image_size
        check("camera frame", frame.shape[:2] == (h, w) and frame.mean() > 30, f"{frame.shape}, mean {frame.mean():.0f}")
        if runtime is not None:
            from lerobot.robots.meca500.autocollect.asr_bridge import AsrTipDetector

            tc = bench.section("target")
            det = AsrTipDetector(runtime, tc["field_centre_px"], tc["field_radius_px"])
            confs, dts = [], []
            for _ in range(N_FRAMES):
                f = robot.get_observation()[bench.camera_key]
                t = time.perf_counter()
                s = det.detect(f)
                dts.append(time.perf_counter() - t)
                confs.append(0.0 if s is None else s.confidence)
            stray = max(confs)
            check(
                "detector on the live feed", (stray < ev.WATCH_MIN_CONFIDENCE) or not at_home,
                f"{N_FRAMES} frames, {np.median(dts):.2f} s/frame, highest confidence {stray:.2f}"
                + ("" if at_home else " (arm not at home: tip may be in view)"),
            )
    except Exception as e:
        check("robot / camera", False, f"{e!r}\n{traceback.format_exc(limit=2)}")
    finally:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass

    failed = [n for n, ok, _ in results if not ok]
    print("\n" + ("GO" if not failed else f"NO-GO: {', '.join(failed)}"))
    sys.stdout.flush()
    os._exit(0 if not failed else 1)  # camera / ONNX threads can hold the interpreter open on Windows


if __name__ == "__main__":
    main()
