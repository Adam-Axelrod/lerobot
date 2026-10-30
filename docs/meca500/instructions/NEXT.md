# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-10-30
**Time needed:** about 1.5 h once the dish holder is in, plus the bead images (step 4, about 15 min)

**Where things stand.**
- **Done:**
  - dry run;
  - hover and touch test runs;
  - MLS203 stage and focus K-Cube connected and readable from Python;
  - a re-calibration on 2026-10-27.
- **Touch runs:** the tip ended inside the marker ring every time, but 100–150 µm off its
  centre. The success threshold is 50 µm.
- **Blocker:** the dish is not on the stage. The glass plate it is meant to sit on is missing,
  so moving the MLS203 does not move the dish. The dish may also have shifted between runs. Until
  it sits in a holder, the 2026-10-27 calibration is provisional, the stage can't randomise the
  background, and no real dataset should be recorded.

**Kinesis.** Only one program can hold the Thorlabs controllers at a time. Close Kinesis before
any step where Claude reads or moves the stage or the focus.

**Do not home the focus K-Cube.** Homing resets its zero and loses the focus (it happened on
2026-10-27). The focus position is saved in `bench.yaml` as `scope.focus_mm`.

## 0. Get the dish holder (no bench time)
1. Ask Huanyu:
   - where the glass plate went;
   - whether it was a Thorlabs part or made for this bench (and if made, whether there is a
     drawing).
2. If it has to be bought: the Thorlabs part is probably the **MLS203P2** (slide / petri-dish
   holder for the MLS203 on inverted microscopes, dishes about 30–60 mm). Check that your dish
   diameter is in its range, and confirm the price and stock with Thorlabs UK.
3. Send Claude a photo of the holder in place, with its height above the stage top if you can
   measure it. The CAD collision model has the stage, but not a holder that might stick up above
   it.

## 1. Fit the holder and refocus (about 15 min)
1. Send the arm home (teleop, **H**) so the pipette is out of the way.
2. Fit the holder and seat the **recording dish** in it (the clean one, with no beads).
3. Open the camera view: `.venv\Scripts\python.exe scripts\meca500\capture_frames.py` (q to quit).
4. In Kinesis, on the focus K-Cube (KST201, serial 26006298), **jog only** (step about
   0.01–0.05 mm near focus) until the scratches and dust on the dish bottom are as sharp as
   possible. A fine pen mark on the underside of the dish makes this easier. Do not press Home.
5. **Close Kinesis**, then tell Claude. Claude reads the new focus position and saves it to
   `bench.yaml`.

## 2. Stage direction and range (Claude runs it, about 5 min)
With the arm at home and Kinesis closed, tell Claude to run the stage-direction measurement. The
stage moves ±0.5 mm in X and Y from (55, 37.5), Claude measures how the picture shifts, and the
stage returns to (55, 37.5). Then tell Claude how far you can move from (55, 37.5) and still
see the dish bottom, clear of the rim (jog in Kinesis, then close it again).

## 3. Re-calibrate, then a touch test (about 30 min)
1. Run `auto_calibrate.py` (`START_ABOVE_DISH_MM = 2.0`, as it is now). See
   "Re-calibrating" below for what a good result looks like.
2. In `scripts/meca500/record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-MM-DD"   # today's date
   STAGE = "touch"
   NUM_EPISODES = 3
   ```
   `touch_offset_mm` in `bench.yaml` is already `0.0` (the tip just touches the dish).
3. Run `.venv\Scripts\python.exe scripts\meca500\record_auto.py` with a hand near the e-stop.
   The tip should come into sharp focus on the marker without visibly sliding.
4. Tell Claude when it has finished. Claude measures each episode's final tip-to-target error
   from the recording; you don't need to send anything unless the terminal shows an error.

What happens next depends on that error. If it is still well over 50 µm, Claude adds a final
vision correction to the recorder (detect the tip just before the touch and nudge it onto the
target) before the real dataset.

## 4. Bead images for the digital target (about 15 min, after step 3)
The real test target is a 0.1 mm zirconia/silica bead, only about 8 px across. These images
set the look of the digital marker and tune the bead detector.

1. Use a **SECOND, clean dish**. Never put beads in the recording dish.
2. Send the arm home. Put a **tiny** amount of beads in the second dish: touch a dry toothpick
   tip to the beads and tap it over the dish. You want a few isolated beads. Keep it dry.
3. Swap it into the holder. **Do not touch the focus or the illumination.**
4. Run `.venv\Scripts\python.exe scripts\meca500\capture_frames.py`. Click the image window,
   then press **space** to save a frame and **q** to quit. Frames go to
   `assets\benches\meca500_basler\beads\`. Save 10–15, moving the stage between them (Kinesis,
   then close it before the next step):
   - about 8 with **one isolated bead** at different places in the image (centre, near the
     edges, near the dark edge of the lit circle);
   - 2–3 with **two or three beads** close together;
   - 2 with **no beads**.
5. Swap the recording dish back. Because the dish was reseated, rerun `auto_calibrate.py`
   before any recording.
6. Tell Claude. Claude commits the frames.

## Send Claude
- step 0: what Huanyu says about the plate, and a photo of the holder in place;
- step 1: "focused, Kinesis closed" (Claude reads the position itself);
- step 2: how far from (55, 37.5) the dish stays usable;
- step 3: the `auto_calibrate` terminal output, and that the touch test finished;
- step 4: that the bead frames are saved, and whether the beads looked sharp.

Claude then rewrites this file with the steps for the real dataset.

## If something goes wrong
| Symptom | What to do |
|---|---|
| `calibration residual ... exceeds` | The script won't run on a poor calibration. Rerun `auto_calibrate.py` (below). |
| `... already holds N episodes` | That `NAME` is already in use. Pick a new one. |
| `No collision-free path home from the current pose` | The arm was left somewhere the model can't plan from. Jog it straight up 25–30 mm with `teleoperate_microscope.py`, press **H**, close teleop, rerun. |
| `Timed out waiting for frame from camera` | Check the microscope lamp is on and the pylon Viewer / `capture_frames.py` is closed, then rerun. |
| The arm starts paused, or won't move | Close teleop, the web portal, and anything on the other PC connected to the robot, then rerun. |
| The robot portal or scripts can't reach `192.168.0.100` | The USB-C Ethernet adapter needs `192.168.0.50/24` (see Claude's memory notes); a different adapter needs it set again in an admin PowerShell. |
| The arm stops with a collision or planning error | Send Claude the output. Nothing has hit anything: every path is checked before the arm moves. |
| Kinesis jog button greyed out | The axis is at the end of its travel (position 0 after homing): only the other direction is allowed. |
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
