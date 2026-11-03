# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-03
**Time needed:** Part A about 1 h. Part B about 45 min, on another day once Claude has updated
the arm's model.

**Where things stand.**
- **Done:**
  - dry run;
  - hover and touch test runs;
  - MLS203 stage and focus K-Cube connected and readable from Python;
  - an acrylic dish holder, flush with the top of the MLS203 (2026-11-03).
- **Touch runs:** the tip ended inside the marker ring every time, but 100–150 µm off its
  centre. The success threshold is 50 µm.
- **The holder raises the dish by about 22 mm.** Everything the arm knows about the dish was
  measured with it down in the stage opening: the dish surface at z = 8.7 mm, where the stage top
  is at z = 31 mm. The focus, the collision model, the touch and start poses and the scope
  calibration are all out of date.

**How this session works.** In Part A the arm stays at home the whole time: you refocus, measure
the holder, and take the bead images. Claude then updates the arm's model, with no bench time.
Part B, the re-calibration and touch test, comes after that.

## Before you start
- **Do not run `auto_calibrate.py` or `record_auto.py`** until Claude says Part B is ready. The
  first move of `auto_calibrate.py` goes 2 mm above the OLD dish height. That point is now about
  20 mm inside the acrylic, and the move is not guarded. Teleop and **H** (home) are fine.
- **Kinesis.** Only one program can hold the Thorlabs controllers at a time. Close Kinesis before
  any step where Claude reads or moves the stage or the focus.
- **Do not home the focus K-Cube.** Homing resets its zero and loses the focus (it happened on
  2026-10-27). The old focus position is 24.08851 mm (`scope.focus_mm` in `bench.yaml`).
- You need both dishes: the clean **recording dish** (never put beads in it) and a **second,
  clean dish** for the beads.

# Part A: today (about 1 h, arm at home throughout)

## A1. Fit the holder (5 min)
1. Open `.venv\Scripts\python.exe scripts\meca500\teleoperate_microscope.py`, press **H** to send
   the arm home, and close teleop.
2. Fit the holder and seat the **recording dish** in it.
3. Take a photo of the holder in place, from the side if you can. Claude needs to see anything
   that sticks up above the stage top.

## A2. Refocus on the dish bottom (15 min)
1. Open the camera view: `.venv\Scripts\python.exe scripts\meca500\capture_frames.py`. Leave it
   open, and don't press space yet.
2. Open Kinesis. On the focus K-Cube (KST201, serial 26006298), **jog only**. Never press Home.
3. The objective has to move about 22 mm, and the actuator has only 25 mm of travel. Jog in
   about 1 mm steps until the picture starts to sharpen. Then use 0.01–0.05 mm steps until the
   scratches and dust on the dish bottom are as sharp as possible. A fine pen mark on the
   underside of the dish helps.
4. **If the jog button greys out before the dish is sharp, stop here.** The actuator is at the end
   of its travel, so the focus can't reach the dish at this height. The holder would need a pocket
   to lower the dish. Write down the position, close Kinesis, and send it to Claude. Skip the rest
   of Part A.
5. Once it's sharp, write down the position Kinesis shows, then **close Kinesis**.
6. Leave `capture_frames.py` open for A4.

## A3. Measure the holder and dish (10 min)
Write these down for Claude:
- the acrylic's thickness;
- whether the dish sits flat on top of the acrylic, or in a pocket (and how deep);
- whether there is a hole under the dish, or the microscope looks through the acrylic;
- the dish's outside diameter and wall height;
- anything on the holder above the stage top (screws, clamps, tabs), and how high it sticks up.

## A4. Stage range (5 min)
Use Kinesis on the MLS203 (BBD302, serial 103476364), jogging only.
1. Start from (55, 37.5).
2. Move the stage in each direction (+X, −X, +Y, −Y) until the dish rim or the edge of the holder
   hole comes into view.
3. Write down each position.
4. Go back to (55, 37.5).
5. Close Kinesis.

## A5. Bead images (20 min)
The real test target is a 0.1 mm zirconia/silica bead, only about 8 px across. These images
set the look of the digital marker and tune the bead detector.

1. Put a **tiny** amount of beads in the **second dish**: touch a dry toothpick tip to the beads
   and tap it over the dish. You want a few isolated beads. Keep it dry.
2. Swap it into the holder in place of the recording dish. **Do not touch the focus or the
   illumination.** If the beads aren't sharp, note it, but don't refocus.
3. In the `capture_frames.py` window, click the image, then press **space** to save a frame. Frames
   go to `assets\benches\meca500_basler\beads\`. Save 10–15, moving the stage between them in
   Kinesis:
   - about 8 with **one isolated bead** at different places in the image (centre, near the
     edges, near the dark edge of the lit circle);
   - 2–3 with **two or three beads** close together;
   - 2 with **no beads**.
4. Press **q** to quit `capture_frames.py`. Move the stage back to (55, 37.5), and close Kinesis.
5. Swap the **recording dish** back into the holder. Leave the arm at home.

## A6. Send Claude
- the focus position from A2 (or where it ran out of travel);
- the measurements from A3, and the photo from A1;
- the four stage positions from A4;
- that the bead frames are saved, and whether the beads looked sharp.

Claude then:
- saves the focus and stage range to `bench.yaml`, and commits the bead frames;
- sets the new dish height, and adds the dish wall to the collision model (it now sticks up above
  the stage, where before it was below it);
- re-solves the touch and start poses. The old ones were chosen for a dish deep in the opening,
  where only 5 of 1512 approach angles were clear;
- runs a planner dry run;
- sets the first re-calibration to start 6 mm up, because the new dish height is only an
  estimate until the tip touches it;
- runs the stage-direction measurement. With Kinesis closed and the arm at home, the stage moves
  ±0.5 mm in X and Y and back while Claude watches the image. Nothing for you to do;
- updates Part B below, and tells you when it is ready.

# Part B: after Claude says it's ready (about 45 min)

## B1. Re-calibrate (15 min)
Run `auto_calibrate.py`, with `START_ABOVE_DISH_MM` as Claude set it (see "Re-calibrating" below).
Keep a hand near the e-stop on the first descent. Send Claude the terminal output and wait for a
go-ahead before B2.

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
| Focus jog greys out before the dish is sharp (A2) | End of travel: the focus can't reach the dish. Stop, note the position, tell Claude. |
| Kinesis jog button greyed out (other times) | The axis is at the end of its travel: only the other direction is allowed. |
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
