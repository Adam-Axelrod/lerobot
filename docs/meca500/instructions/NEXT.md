# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-03
**Time needed:** about 2 h at the bench, plus the bead images (step 4, about 15 min)

**Where things stand.**
- **Done:**
  - dry run;
  - hover and touch test runs;
  - MLS203 stage and focus K-Cube connected and readable from Python;
  - a re-calibration on 2026-10-27;
  - **an acrylic dish holder** (2026-11-03), flush with the top of the MLS203.
- **Touch runs:** the tip ended inside the marker ring every time, but 100–150 µm off its
  centre. The success threshold is 50 µm.
- **The dish is now about 22 mm higher.** All the arm's settings were measured with the dish down
  in the stage opening: the calibrated dish surface is z = 8.7 mm, and the stage top is z = 31 mm.
  The collision model, the touch and start poses, the scope calibration and the focus are all
  out of date until steps 0 and 1 below are done.

**Until Claude has done step 1, do not run `auto_calibrate.py` or `record_auto.py`.**
`auto_calibrate.py` first drives the tip to 2 mm above the OLD dish height. That point is now
about 20 mm inside the acrylic holder, and that move is not guarded. Teleop and **H** (home) are
fine.

**Kinesis.** Only one program can hold the Thorlabs controllers at a time. Close Kinesis before
any step where Claude reads or moves the stage or the focus.

**Do not home the focus K-Cube.** Homing resets its zero and loses the focus (it happened on
2026-10-27). The old focus position is saved in `bench.yaml` as `scope.focus_mm` (24.08851 mm).

## 0. Fit the holder and check the focus can reach the dish (about 20 min)
1. Send the arm home (teleop, **H**) so the pipette is out of the way.
2. Fit the holder and seat the **recording dish** in it (the clean one, with no beads).
3. Open the camera view: `.venv\Scripts\python.exe scripts\meca500\capture_frames.py` (q to quit).
4. In Kinesis, on the focus K-Cube (KST201, serial 26006298), **jog only** until the scratches and
   dust on the dish bottom are as sharp as possible. Do not press Home. The objective has to move
   about 22 mm, and the ZST225 has only 25 mm of travel, so:
   - use big steps (about 1 mm) until the picture starts to sharpen, then 0.01–0.05 mm near focus;
     a fine pen mark on the underside of the dish makes this easier;
   - **if the jog button greys out (end of travel) before the dish is sharp, stop.** The focus
     can't reach the dish at this height, and the holder needs a pocket to lower it. Tell Claude
     the position where it stopped.
5. **Close Kinesis**, then tell Claude. Claude reads the new focus position and saves it to
   `bench.yaml`.
6. Measure the following and send them to Claude, with a photo of the holder in place:
   - the acrylic's thickness;
   - whether the dish sits flat on top of the acrylic, or in a pocket (and how deep);
   - whether there is a hole under the dish, or the microscope looks through the acrylic;
   - the dish's outside diameter and wall height;
   - anything on the holder that sticks up above the stage top (clamps, screws, tabs).

## 1. Claude updates the arm's model (no bench time)
From step 0's numbers, Claude:
- sets the new dish height;
- adds the dish wall to the collision model (it now sticks up above the stage, where before it
  was below it);
- re-solves the touch and start poses. The old ones were chosen for a dish deep in the opening,
  where only 5 of 1512 approach angles were clear;
- runs a dry run of the planner, and sets the first re-calibration to start 6 mm up, because
  the new dish height is only an estimate until the arm touches it.

Claude then tells you when steps 2 onwards are safe to run.

## 2. Stage direction and range (Claude runs it, about 5 min)
With the arm at home and Kinesis closed, tell Claude to run the stage-direction measurement. The
stage moves ±0.5 mm in X and Y from (55, 37.5), Claude measures how the picture shifts, and the
stage returns to (55, 37.5). Then tell Claude how far you can move from (55, 37.5) and still
see the dish bottom, clear of the rim (jog in Kinesis, then close it again).

## 3. Re-calibrate, hover check, then a touch test (about 45 min)
1. Run `auto_calibrate.py`, with `START_ABOVE_DISH_MM` as Claude set it in step 1. See
   "Re-calibrating" below for what a good result looks like. Send Claude the terminal output
   before going on: the new touch pose is checked by a hover run first.
2. Hover check: in `scripts/meca500/record_auto.py`, set `NAME = "basler_hover_2026-MM-DD"`
   (today's date), `STAGE = "hover"` and `NUM_EPISODES = 3`, and run it with a hand near the
   e-stop. The tip should stop just above the marker without touching. Tell Claude when it is done.
3. In `scripts/meca500/record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-MM-DD"   # today's date
   STAGE = "touch"
   NUM_EPISODES = 3
   ```
   `touch_offset_mm` in `bench.yaml` is already `0.0` (the tip just touches the dish).
4. Run `.venv\Scripts\python.exe scripts\meca500\record_auto.py` with a hand near the e-stop.
   The tip should come into sharp focus on the marker without visibly sliding.
5. Tell Claude when it has finished. Claude measures each episode's final tip-to-target error
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
- step 0: "focused, Kinesis closed" (Claude reads the position itself), or where the focus ran
  out of travel; the holder and dish measurements, and a photo of the holder in place;
- step 2: how far from (55, 37.5) the dish stays usable;
- step 3: the `auto_calibrate` terminal output, then that the hover check and the touch test
  finished;
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
