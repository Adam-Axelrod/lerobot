# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-03
**Time needed:** about 15 min (B4).

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
  done: +X moves the image down, +Y moves it right, 12.85 µm/px. Stage range measured: the dish
  is centred under the camera at stage (50.3, 34.8) and is usable over about ±10 mm.
- **Model ready:** the dry run plans 100/100 episodes, tightest 1.3 mm beyond the margin.
- **2026-11-03 incident:** `record_auto.py` ran before the model was updated and bent the
  pipette on the raised dish. `record_auto.py` and `auto_calibrate.py` now refuse to start unless
  the focus K-Cube reads `scope.focus_mm`. This needs the K-Cube on, plugged in by USB, and
  Kinesis closed.

## Before you start
- **Inspect the pipette** (it bent on 2026-11-03). Replace it if it is bent, cracked or chipped.
  The re-calibration starts 6 mm up either way.
- **Keep the stage at (55, 37.5)** for all of Part B. The collision model places the dish for
  that stage position only; random stage moves come later, once Claude has written the stage
  driver.
- **Kinesis.** Only one program can hold the Thorlabs controllers at a time. Close Kinesis before
  any step where Claude reads or moves the stage or the focus.
- **Do not home or move the focus K-Cube.** It must stay at 4.000 mm.

# Part B: B4 now (about 15 min)

## B1. Re-calibrate: DONE (2026-11-03)
Tip moved ~1.2 mm (the pipette shifted when it bent); pivot 0.076 mm, scope 1.26 px (16 µm),
dish z 26.83, verify ≤ 1 px. The collision model now uses the measured dish height, and the dry
run still plans 100/100. `START_ABOVE_DISH_MM` is back to 2.0.

## B2-B3: DONE (2026-11-03)
Hover and open-loop touch tests ran cleanly. The touches landed 62-122 µm from the target (mean
~95), above the 50 µm threshold, so Claude added a vision correction to `record_auto.py`
(`VISION_CORRECTION = True`): on the final approach the arm pauses twice (at the pre-touch point,
and 0.4 mm before the touch), detects the tip, and shifts the rest of the approach onto the
target. The pauses are part of the recorded episode.

## B4. Touch test with the vision correction (15 min)
1. Keep the stage at (55, 37.5) and Kinesis closed.
2. In `scripts\meca500ecord_auto.py`, set:
   ```python
   NAME = "basler_touch_corrected_2026-MM-DD"   # today's date
   STAGE = "touch"
   NUM_EPISODES = 3
   ```
3. Run it with a hand near the e-stop. On the way in, the arm stops twice, for about half a second
   each time. It may then move very slightly sideways, by up to ~0.1 mm, before touching.
4. The terminal prints `correction at pretouch`, `correction at near` and `final: ... um` for each
   episode. `tip not detected` at the pretouch checkpoint is normal when the pre-touch point is
   outside the image.
5. Tell Claude when it has finished. Claude checks the final errors against 50 µm.

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
| `Episode N discarded: ... would collide` | A correction move failed its collision check; the next episode carries on. Send Claude the output if it happens often. |

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
