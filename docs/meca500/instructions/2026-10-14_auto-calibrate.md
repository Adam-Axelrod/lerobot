# Auto-calibrate after a pipette change or knock

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Time needed:** about 10–15 min, hands off
**Replaces:** `calibrate_tip.py` + `import_bench_cad.py` + `calibrate_scope.py` (still there
as a manual fallback).
Background: [`../autocalibration_plan.md`](../autocalibration_plan.md).

## What it does
`scripts/meca500/auto_calibrate.py` runs five stages by itself, showing everything on the
microscope feed:

| Stage | What happens |
|---|---|
| 1. Home + background | Drives home and grabs a background image with the tip out of view |
| 2. Find the tip | Hovers 2 mm over the dish height found by the last calibration (`START_ABOVE_DISH_MM`), at the field-of-view centre, and looks for the tip (spiral-searching sideways if needed). Once seen it centres it and **descends in view** until it touches the dish (see *How it senses contact* below). Only if the tip cannot be seen does it find the dish blind first and search just above it. |
| 3. Tip calibration | At up to 9 wrist orientations (tilts and twists of 7–12°, skipping any the collision model forbids), steers the tip back over the same image point and touches. A pivot fit gives the true tip, which is saved and the tool model re-imported. |
| 4. Scope calibration | Touches about 12 points over the visible field, detects the tip at each, and fits the pixel ↔ position map. The old file is backed up. |
| 5. Verify | Touches 3 fresh points and reports how far the tip lands from where the new calibration predicts. Then returns home. |

## On screen
| Marker | Meaning |
|---|---|
| Magenta **x** | Where the camera detects the tip |
| Yellow **+** | The current target |
| Cyan **+** | Where the new calibration predicts the tip (verify stage) |
| Green circles | Touches recorded so far: where the tip touched down, one per touch. In stage 3 they should all sit on the same spot; in stage 4 they spread over the field |
| Top banner | **Red CONTACT** (and what sensed it: `vision: tip started sliding`, or the force backstop) right after a touch. **Green** while a descent is watching for a touch, with the backstop torque bar under it. **Grey** otherwise: while the arm moves or hovers it is not sensing, so the force reading (swinging with the arm's motion) is ignored |
| Black strip at the bottom | What it's doing right now |

The window title shows the stage (`1/5` … `5/5`) with a one-line description of it, plus a
legend for the markers.

## How it senses contact
The pipette meets the dish at a shallow angle, so the instant it touches it starts **sliding
forward** along the dish. On the arm the torque only crossed its threshold after a millimetre or
more of sliding. So the camera watches for the slide: while descending, the tip barely moves in
the image; once touching it moves forward about 100 px per mm of descent. The script fits that
"flat, then rising" shape and takes the touch-down point, both the pixel and the arm pose, from
the bend, not from where the tip ended up.

The force sensor is now only a **backstop**, with high thresholds (1.5 mNm torque, 0.2 N). At the
old 0.3 mNm setting, sensor drift during a long descent triggered it in mid-air (2026-10-16).
The backstop catches the blind fallback (tip not visible) and a missed slide. The terminal says
which one fired: `contact (vision: tip started sliding)` normally, or `contact (torque)` /
`contact (force limit)` if the backstop stopped it. Tell Claude if you see a backstop stop. Press **q** in the window (or close it) to
abort: the arm stops and holds its pose, and nothing further is saved.

## Steps
0. **After a pipette change or a knock** (not a routine recalibration): set
   `START_ABOVE_DISH_MM = 6.0` in the CONFIG block of `auto_calibrate.py` for this run. The new
   tip may sit several mm lower than the last calibration says, and the move to the start
   position is not guarded. Set it back to 2.0 afterwards.
1. Close teleop and the web portal. Put a dish on the stage. Hand near the **e-stop**.
2. Run:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\auto_calibrate.py
   ```
3. **Watch the first run closely**, especially:
   - the first descent: the banner must turn **red** and the arm stop the moment the tip
     touches;
   - the tip search: the magenta **x** should land on the tip end, not on the side of the
     pipette;
   - the wrist rotations in stage 3: the arm turns at 3 mm above the dish.
4. At the end the terminal prints **AUTO-CALIBRATION COMPLETE** with:
   - the measured tip (saved to `bench.yaml` → `tool.measured_tip_in_flange`);
   - the scope fit, in px and µm (saved to `scope_calibration.json`);
   - the verify errors.
5. **Send Claude the terminal output.**

## What a good result looks like
- a pivot-fit residual of about 0.1 mm or less;
- a scope fit of about 3 px or less;
- verify errors within a few px.

In the offline simulation (`sim_auto_calibrate.py`), with the tip sliding on contact, it
recovered a tip that was 3.3 mm off to within 0.02 mm, with verify errors under 1 px. Real images and touches will be noisier.

## If something goes wrong
| Message | What it means / what to do |
|---|---|
| `CALIBRATION FAILED: Could not find the tip ...` | The tip wasn't seen even after a 6 mm spiral search. Check the camera exposure and focus, and that the dish is in the light path. |
| `... Servo did not converge` / `Lost the tip` | The tip detection is unreliable on this pipette. Send Claude the output; a screenshot of the window helps. |
| `Only N usable pivot touches` | Too many wrist orientations collide or failed. Send the output. |
| `No contact before the over-travel limit` | The arm descended 2.5 mm past where the dish should be without feeling it. Check the sensor (the torque bar should move) and that a dish is present. |
| `contact (torque)` or `contact (force limit)` in the terminal | The force backstop stopped a descent, not the slide. Tell Claude, especially if the tip was still in the air. |

Nothing is saved if the run fails or is aborted before stage 3 ends. If it fails during stage
4, the new tip is already saved but the old scope calibration is untouched. Just rerun.
