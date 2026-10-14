# Meca500 docs (this fork)

Fork-specific documentation for the Meca500 microscope-pipetting work. Upstream LeRobot's own
documentation lives in `docs/source/` and is built by Hugging Face's CI; keep Meca500 material
out of it.

| Path | What it is |
|---|---|
| [`autocollect_plan.md`](autocollect_plan.md) | Design and status of the automated, collision-checked dataset collection |
| [`autocalibration_plan.md`](autocalibration_plan.md) | Design of the automatic tip + scope calibration (vision + Bota contact sensing) |
| [`instructions/`](instructions/) | Step-by-step checklists for work at the bench, newest last |

## Instructions, in order

Each file is a self-contained checklist for one session at the arm. File names start with the
date they were written.

| File | Bench | Status |
|---|---|---|
| [`2026-10-14_scope-calibration-and-first-hover-run.md`](instructions/2026-10-14_scope-calibration-and-first-hover-run.md) | Huanyu's setup | Done: hover run recorded 3 episodes, but the arm went limp on disconnect and bent the pipette (fixed) |
| [`2026-10-14_after-pipette-crash-recalibrate.md`](instructions/2026-10-14_after-pipette-crash-recalibrate.md) | Huanyu's setup | Step 1 (moved) and Part A done (tip 0.065 mm rms, imported); **Part B next** |
| [`2026-10-14_bota-force-probe.md`](instructions/2026-10-14_bota-force-probe.md) | Huanyu's setup | Done: torque, not force, sees a touch (0.3-1 mNm steps; force buried in noise) |
| [`2026-10-14_auto-touch-test.md`](instructions/2026-10-14_auto-touch-test.md) | Huanyu's setup | **To do** (after the scope recalibration) |
| [`2026-10-14_auto-calibrate.md`](instructions/2026-10-14_auto-calibrate.md) | Huanyu's setup | **To do** (after the auto-touch test passes) |
