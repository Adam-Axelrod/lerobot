# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-10-22
**Time needed:** about 45 min

**Where things stand.** `auto_calibrate.py` ran again on 2026-10-16. The tip moved less than
0.1 mm. The scope fit is 1.15 px (15 µm), and the dish surface is at z 8.703. Since the pipette
crash, no automated recording has run with this calibration. This session reruns the hover test,
then does the first touch runs. The real dataset comes after that.

> **Do not move or swap the dish.** The dish height was measured on the dish that is on the
> stage now. If you use a different dish or reseat this one, rerun `auto_calibrate.py` first
> (see the end of this file).

Every setting below is in a CONFIG block at the top of the named script.

## 1. Commit the URDF from the Windows laptop (2 min)
The planner and collision code read `mecademic_description/urdf/meca_arm_only.urdf`. That file
only exists on the Windows laptop: it was gitignored and never committed. The ignore rule now
makes an exception for it, so commit it once:
```powershell
git pull
git add mecademic_description/urdf/meca_arm_only.urdf
git commit -m "chore(meca500): track the arm-only URDF the planner uses"
git push
```
After that, the planner and collision tests can run on the Mac.

## 2. Dry run (no robot, 1 min)
Check that the planner still plans with the 2026-10-16 dish height:
```powershell
.venv\Scripts\python.exe scripts\meca500\plan_preview.py
```
- Every one of the 100 episodes should plan. Note the tightest clearance it reports (it was
  0.5 mm beyond the margin with the old calibration).
- Open `assets\benches\meca500_basler\preview.html` and check that no path runs near the
  illuminator column.

## 3. Hover re-run (3 episodes)
The tip stops 5 mm above the dish, at half speed. This rechecks the hover run with the new tip.

1. Start with the arm at home and the pipette lifted out of the dish. Close teleop and the web
   portal.
2. In `scripts/meca500/record_auto.py`, set:
   ```python
   NAME = "basler_hover_2026-10-22"   # use the date of the run; test runs never go in the training dataset
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

## 4. First touch runs (3 + 3 episodes)
The tip now goes all the way down to the dish on the marker.

1. In `assets/benches/meca500_basler/bench.yaml`, set `target.touch_offset_mm: 0.1`. It is
   currently `0.0`. At 0.1 the tip stops 0.1 mm above the dish: in focus, but not pressing.
2. In `record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-10-22"
   STAGE = "touch"
   ```
   Keep `NUM_EPISODES = 3`.
3. Run it the same way. At the end of each episode:
   - the tip should come into sharp focus **on the marker**;
   - the pipette should not visibly slide or bend.
4. If that looks right, set `touch_offset_mm: 0.0`. Then run 3 more episodes with
   `NAME = "basler_touch_test2_2026-10-22"`. This time the tip should just touch the dish.

## Send Claude
- the terminal output of `plan_preview.py` and of each recording run;
- for each run, whether the tip ended on the marker (a screenshot of the last frame helps);
- anything that came close to the illuminator.

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
