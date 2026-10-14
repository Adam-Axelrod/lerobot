# First automated runs after auto-calibration: hover, then touch

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Time needed:** about 30 min
**Before this:** [`2026-10-14_auto-calibrate.md`](2026-10-14_auto-calibrate.md), done on
2026-10-14. Results: tip fit 0.067 mm rms; scope fit 1.09 px (14 µm); verify ≤ 2.2 px.
The offline dry run (`plan_preview.py`) plans all 100 episodes with the new tip. The tightest
clearance is 0.5 mm beyond the margin to the illuminator.

> **Do not move or swap the dish.** The dish height (z 8.807) was measured on the dish that is
> on the stage now. A different dish, or this one reseated, needs `auto_calibrate.py` again
> (about 10 min, hands off).

All settings are in the CONFIG block at the top of `scripts/meca500/record_auto.py`.

## Part A: hover re-run (3 episodes)
The tip stops 5 mm above the dish at half speed. This was fine before the crash, so this run only
re-checks it with the new tip.

1. Set:
   ```python
   NAME = "basler_hover_2026-10-14"   # test runs never go in the training dataset
   NUM_EPISODES = 3
   STAGE = "hover"
   ```
   (The old `basler_auto_2026-10` dataset holds the 3 pre-crash hover episodes. The script
   refuses to overwrite it, which is why the name changes.)
2. Hand near the **e-stop**, then run:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\record_auto.py
   ```
3. Watch for:
   - no part of the arm or pipette coming near the illuminator column;
   - the blurred tip ending each episode over the marker. It is 5 mm out of focus, so expect a
     blur, not a point.

## Part B: first touch run (3 episodes)
Now the tip goes all the way down to the dish on the marker.

1. In `assets/benches/meca500_basler/bench.yaml`, set `target.touch_offset_mm: 0.1`. The first
   touch runs then stop 0.1 mm above the dish: in focus, but not pressing.
2. In `record_auto.py`, set:
   ```python
   NAME = "basler_touch_test_2026-10-14"
   STAGE = "touch"
   ```
   Keep `NUM_EPISODES = 3`.
3. Run as before. Watch the end of each episode:
   - the tip should come into sharp focus **on the marker**;
   - the pipette should not visibly slide or bend.
4. If that looks right, set `touch_offset_mm: 0.0` and run 3 more episodes with
   `NAME = "basler_touch_test2_2026-10-14"`. The tip should now just kiss the dish.

## Send Claude
- the terminal output of each run;
- for each part, whether the tip ended on the marker (a screenshot of the last frame helps);
- anything that looked close to the illuminator.

The next step after that is the real dataset. The old `basler_auto_2026-10` folder in
`~\.cache\huggingface\lerobot\AdamAxelrod\` holds the 3 pre-crash hover episodes, and the script
refuses to write into a folder that already holds episodes. Either delete that folder or pick a
new NAME for the real dataset.

## If something goes wrong
| Symptom | What to do |
|---|---|
| `calibration residual ... exceeds` | Refuses to run on a poor calibration. Rerun `auto_calibrate.py`. |
| The arm stops with a collision or planning error | Send the output. Nothing has touched anything: every path is checked before it moves. |
| The tip is in focus but off the marker by more than ~10 px | The tip or scope calibration drifted (for example, the pipette was knocked). Rerun `auto_calibrate.py`. |
| The pipette slides at the end of a touch episode | Press **Esc**, set `touch_offset_mm` back to 0.1 and tell Claude. |
