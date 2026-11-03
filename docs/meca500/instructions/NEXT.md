# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-03
**Time needed:** Part A about 10 min. Part B about 45 min, after Claude says it's ready.

**Where things stand.**
- **Done:**
  - acrylic dish holder fitted, photographed and measured;
  - refocused on the dish bottom: focus K-Cube at **4.000 mm** (saved as `scope.focus_mm`);
  - arm model updated by Claude for the raised dish: dish and acrylic added to the collision
    model, new touch and start poses, a dish-height estimate of z 27.
- **Dropped for now:** the zirconia beads. The dataset uses the digital target over the empty
  dish; the real-bead test comes later.
- **Stage working:** after a BBD302 power cycle, both axes enabled from Python (the X axis had a
  latched fault before). Both are homed and at (55, 37.5). The stage-direction measurement is
  done: +X moves the image down, +Y moves it right, 12.85 µm/px.
- **2026-11-03 incident:** `record_auto.py` ran before the model was updated and bent the
  pipette on the raised dish. `record_auto.py` and `auto_calibrate.py` now refuse to start unless
  the focus K-Cube reads `scope.focus_mm`. This needs the K-Cube on, plugged in by USB, and
  Kinesis closed.

## Before you start
- **Do not run `auto_calibrate.py` or `record_auto.py`** until Claude says Part B is ready.
  Teleop and **H** (home) are fine.
- **Kinesis.** Only one program can hold the Thorlabs controllers at a time. Close Kinesis before
  any step where Claude reads or moves the stage or the focus.
- **Do not home or move the focus K-Cube.** It must stay at 4.000 mm.

# Part A: now (about 10 min, arm at home throughout)

## A1. Stage range (5 min)
Use Kinesis on the MLS203 (BBD302, serial 103476364), jogging only. Both axes are already enabled
and homed; if Kinesis shows them disabled (e.g. after a power cycle), Enable and Home both first,
with the arm at home.
1. Start from (55, 37.5), with the camera view open
   (`.venv\Scripts\python.exe scripts\meca500\capture_frames.py`, q to quit).
2. Move the stage in each direction (+X, −X, +Y, −Y) until the dish wall comes into view.
3. Write down each position.
4. Go back to (55, 37.5) and **close Kinesis**.

## A2. Send Claude
- the four stage positions from A1;
- whether the pipette was replaced, or looked undamaged.

Claude then:
- widens the dish in the collision model to cover the stage range;
- tells you Part B is ready.

# Part B: after Claude says it's ready (about 45 min)

## B1. Re-calibrate (15 min)
1. Check the stage is at (55, 37.5) and Kinesis is closed.
2. Run `.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py`. It is set to start 6 mm above
   the ESTIMATED dish height (z 27). Keep a hand near the e-stop on the first descent.
3. Send Claude the terminal output and wait for a go-ahead before B2. Claude moves the dish in
   the collision model to the measured height.

## B2. Hover check (10 min)
1. In `scripts\meca500\record_auto.py`, set:
   ```python
   NAME = "basler_hover_2026-MM-DD"   # today's date
   STAGE = "hover"
   NUM_EPISODES = 3
   ```
2. Run `.venv\Scripts\python.exe scripts\meca500\record_auto.py` with a hand near the e-stop.
   The tip should stop just above the marker without touching.
3. Tell Claude when it's done.

## B3. Touch test (15 min)
1. In `record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-MM-DD"   # today's date
   STAGE = "touch"
   NUM_EPISODES = 3
   ```
   `touch_offset_mm` in `bench.yaml` is already `0.0` (the tip just touches the dish).
2. Run it again with a hand near the e-stop. The tip should come into sharp focus on the marker
   without visibly sliding.
3. Tell Claude when it's done. Claude measures each episode's final tip-to-target error from the
   recording.

What happens next depends on that error. If it is still well over 50 µm, Claude adds a final
vision correction to the recorder (detect the tip just before the touch and nudge it onto the
target) before the real dataset. Then Claude rewrites this file with the steps for recording the
real dataset.

## If something goes wrong
| Symptom | What to do |
|---|---|
| `REFUSING TO START: Focus K-Cube ... not found` | Power the focus K-Cube, plug in its USB, close Kinesis, rerun. |
| `REFUSING TO START: The microscope focus is at ... but the bench was calibrated at ...` | The dish or focus changed since calibration. Don't override it: tell Claude. |
| A stage axis won't enable in Kinesis | Power-cycle the BBD302, close Kinesis and ask Claude to enable it from Python (that worked on 2026-11-03). |
| Kinesis jog button greyed out | The axis is at the end of its travel: only the other direction is allowed. |
| `capture_frames.py` or a script can't open the camera | Close the pylon Viewer and any other script using the Basler, then rerun. |
| `calibration residual ... exceeds` | The script won't run on a poor calibration. Rerun `auto_calibrate.py` (below). |
| `... already holds N episodes` | That `NAME` is already in use. Pick a new one. |
| `No collision-free path home from the current pose` | The arm was left somewhere the model can't plan from. Jog it straight up 25–30 mm with `teleoperate_microscope.py`, press **H**, close teleop, rerun. |
| `Timed out waiting for frame from camera` | Check the microscope lamp is on and the pylon Viewer / `capture_frames.py` is closed, then rerun. |
| The arm starts paused, or won't move | Close teleop, the web portal, and anything on the other PC connected to the robot, then rerun. |
| The robot portal or scripts can't reach `192.168.0.100` | The USB-C Ethernet adapter needs `192.168.0.50/24` (see Claude's memory notes); a different adapter needs it set again in an admin PowerShell. |
| The arm stops with a collision or planning error | Send Claude the output. Nothing has hit anything: every path is checked before the arm moves. |
| The pipette slides at the end of a touch episode | Press **Esc**. Set `touch_offset_mm` to 0.1 and tell Claude. |

## Re-calibrating (after the dish is reseated, the focus changes, or the pipette is knocked)
This takes about 10–15 min and runs hands off:
```powershell
.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py
```
- **After a pipette change or a knock**, first set `START_ABOVE_DISH_MM = 6.0` in its CONFIG
  block, because the move to the start position is not guarded. Set it back to 2.0 afterwards.
- Watch the first descent: the banner should turn red and the arm should stop the moment the tip
  touches.
- A good result has a pivot residual of about 0.1 mm or less, a scope fit of about 3 px or less,
  and verify errors of a few px. Send Claude the terminal output.
- If the terminal says `contact (torque)` or `contact (force limit)`, the backstop stopped the
  descent instead of the slide detection. Tell Claude.
