# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-10-27
**Time needed:** about 45 min for steps 1-3, about 20 min for step 4, about 15 min for step 5

**Order.** Do steps 1-3 first, on the calibrated dish. Step 4 can happen at any time. Do step 5
last, because it uses a second dish.

**Where things stand.** `auto_calibrate.py` ran again on 2026-10-16. The tip moved less than
0.1 mm. The scope fit is 1.15 px (15 µm), and the dish surface is at z 8.703. Since the pipette
crash, no automated recording has run with this calibration. This session reruns the hover test,
then does the first touch runs. The real dataset comes after that.

> **Do not move or swap the dish.** The dish height was measured on the dish that is on the
> stage now. If you use a different dish or reseat this one, rerun `auto_calibrate.py` first
> (see the end of this file).

Every setting below is in a CONFIG block at the top of the named script.

## 1. Dry run (no robot, 1 min)
Check that the planner still plans with the 2026-10-16 dish height:
```powershell
.venv\Scripts\python.exe scripts\meca500\plan_preview.py
```
- Every one of the 100 episodes should plan. Note the tightest clearance it reports (it was
  0.5 mm beyond the margin with the old calibration).
- Open `assets\benches\meca500_basler\preview.html` and check that no path runs near the
  illuminator column.

## 2. Hover re-run (3 episodes)
The tip stops 5 mm above the dish, at half speed. This rechecks the hover run with the new tip.

1. Start with the arm at home and the pipette lifted out of the dish. Close teleop and the web
   portal.
2. In `scripts/meca500/record_auto.py`, set:
   ```python
   NAME = "basler_hover_2026-10-27"   # use the date of the run; test runs never go in the training dataset
   NUM_EPISODES = 3
   STAGE = "hover"
   ```
   `NAME` must change. The old `basler_auto_2026-10` dataset still holds the 3 hover episodes
   from before the crash, and the script refuses to write into it.
3. Keep a hand near the **e-stop**, then run:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\record_auto.py
   ```
4. Watch for these:
   - nothing on the arm or pipette comes near the illuminator column;
   - each episode ends with the blurred tip over the green marker. The tip is 5 mm out of
     focus, so expect a blur, not a sharp point.

## 3. First touch runs (3 + 3 episodes)
The tip now goes all the way down to the dish on the marker.

1. In `assets/benches/meca500_basler/bench.yaml`, set `target.touch_offset_mm: 0.1`. It is
   currently `0.0`. At 0.1 the tip stops 0.1 mm above the dish: in focus, but not pressing.
2. In `record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-10-27"
   STAGE = "touch"
   ```
   Keep `NUM_EPISODES = 3`.
3. Run it the same way. At the end of each episode:
   - the tip should come into sharp focus **on the marker**;
   - the pipette should not visibly slide or bend.
4. If that looks right, set `touch_offset_mm: 0.0`. Then run 3 more episodes with
   `NAME = "basler_touch_test2_2026-10-27"`. This time the tip should just touch the dish.

## 4. Connect the MLS203 stage (any time, about 20 min)
This is independent of steps 1-3. `record_auto.py` will move the dish to a random position
between episodes, so the policy never sees the same patch of dish twice. The scope calibration
maps camera pixels to robot positions, and moving the dish under the fixed camera does not change
that mapping.

1. **Install Thorlabs Kinesis, 64-bit**, from thorlabs.com (Software → Motion Control →
   Kinesis), to the default folder. The code will expect `C:\Program Files\Thorlabs\Kinesis\`.
   This PC's Python is 64-bit, so the 32-bit build will not load.
2. **Raise the pipette first**: send the arm home. Homing drives the stage to its limits and
   carries the dish with it, so it must not drag under the tip.
3. Connect the stage controller by USB and power it on. Open **Kinesis**; the controller should
   appear with two channels (X and Y). Note the controller **model** from its front label
   (probably a BBD302) and its **serial number** (a BBD302's starts with `103`).
4. **Home both channels**, then jog each axis by about 1 mm while watching the microscope feed.
   Note which way the image moves for +X and for +Y.
5. Move the stage until a clear part of the dish is under the objective, and note that
   **X, Y position in mm**. This will be the centre that the random positions are spread around.
   Check how far you can move from it (try ±5 mm per axis) and stay well inside the dish, clear
   of the rim and any labels.
6. **Close Kinesis.** Only one program can hold the controller, so the app must be closed before
   the Python scripts can connect.

Claude then adds `pythonnet`, a small stage driver and a `stage_probe.py` test script. The first
test runs against the Kinesis simulator, with no hardware involved.

## 5. Bead images for the digital target (do this LAST, about 15 min)
The real test target is a 0.1 mm zirconia/silica bead. At our scale that is only about 8 px
across. The digital marker will be made to look like a bead, from crops of these images, and the
same images tune the detector that finds the real bead at test time.

1. **Use a SECOND, clean dish**, of the same type as the calibrated one. Never put beads in the
   calibrated dish: stray beads would show up in every recorded episode, and the tip could hit
   them.
2. Send the arm home so the pipette is out of view.
3. Put a **tiny** amount of beads in the second dish: touch a dry toothpick tip to the beads and
   tap it over the dish. You want a few isolated beads, not a carpet. Keep the dish dry, as in
   recording.
4. Swap the dish onto the stage. **Do not touch the microscope focus or the illumination**; they
   must match recording.
5. Close the pylon Viewer, then run:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\capture_frames.py
   ```
   Click the image window, then press **space** to save a frame and **q** to quit. Frames go to
   `assets\benches\meca500_basler\beads\`. Save 10-15 frames, moving the dish between them
   (by hand, or with the stage from step 4):
   - about 8 with **one isolated bead** at different places in the image (centre, near the
     edges, near the dark edge of the lit circle);
   - 2-3 with **two or three beads** close together;
   - 2 with **no beads** (background only).

   If beads look out of focus because they sit on the dish bottom and the focus is set
   elsewhere, still save them, and tell Claude.
6. Put the calibrated dish back. Its height will have changed slightly, so the next session
   starts with `auto_calibrate.py` (about 10-15 min, hands off; see the end of this file).
7. Commit and push the frames (`git add assets/benches/meca500_basler/beads`), or tell Claude
   and it will.

## Send Claude
- the terminal output of `plan_preview.py` and of each recording run;
- for each run, whether the tip ended on the marker (a screenshot of the last frame helps);
- anything that came close to the illuminator.
- for step 4: the controller model and serial number, the centre X, Y in mm, how far you can
  move from it and stay inside the dish, the image direction for +X and +Y, and whether homing
  worked.
- for step 5: that the bead frames are saved, how many, and whether the beads looked sharp.

Claude then rewrites this file with the steps for the real dataset.

## If something goes wrong
| Symptom | What to do |
|---|---|
| `calibration residual ... exceeds` | The script won't run on a poor calibration. Rerun `auto_calibrate.py` (below). |
| `... already holds N episodes` | That `NAME` is already in use. Pick a new one. |
| The arm starts paused, or won't move | Close teleop and the web portal, then rerun. The script resumes a paused arm when it connects. |
| The arm stops with a collision or planning error | Send Claude the output. Nothing has hit anything: every path is checked before the arm moves. |
| The tip is in focus but more than ~10 px off the marker | The tip or scope calibration has drifted, for example because the pipette was knocked. Rerun `auto_calibrate.py`. |
| The pipette slides at the end of a touch episode | Press **Esc**. Set `touch_offset_mm` back to 0.1 and tell Claude. |

## Re-calibrating (only if the dish moved or the pipette was knocked)
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
