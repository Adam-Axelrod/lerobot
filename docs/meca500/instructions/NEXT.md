# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-04
**Time needed:** about 1.5 h per 50-episode batch, mostly hands off.

**Where things stand.**
- **D2 (20 touch episodes with stage moves) ran cleanly:** no discards, no dish sliding.
  - 14 of the 18 measured episodes were within 50 µm (median 22 µm).
  - **Two of the four misses were false readings, not real misses.** In episodes 1 and 10 the
    video shows the tip in the ring, but the tip detector placed it 16 px (~210 µm) past its real
    end. This seems to happen only with the tip on the dish.
  - **Episode 8:** a large piece of debris next to the tip fooled the detector at both checkpoints.
    The 0.5 mm safety cap skipped both corrections, as intended.
  - **Episode 9:** looks like a real 80 µm miss.
- **`record_auto.py` now saves the raw frames** (no target ring) behind every measurement, so Claude
  can fix the detector and re-score the final errors offline. The demonstrations themselves look
  good, so the real dataset can start.
- **D1 height check (12:36):** dish within 3 µm of C1 (11:27): no Blu-tack creep so far.
- Calibration (C1, 2026-11-04): pivot 0.057 mm, scope 1.18 px, dish z 28.55, height-map tilt 0.11°.

## Before you start
- **Close Kinesis.** The scripts drive the stage and read the focus K-Cube themselves; only one
  program can hold the controllers. Both controllers must be on and plugged in by USB.
- **Do not home or move the focus K-Cube.**
- **Debris (optional):** if there is a large loose particle in the dish, blow it out gently with an
  air duster without touching the dish. Then run E0.
- Keep a hand near the e-stop for the first few episodes.

## E0. Height check (about 3 min), before every batch
`HEIGHT_CHECK_ONLY = True` is already set in `auto_calibrate.py`. Run
`.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py` and check that it ends with
`HEIGHT CHECK COMPLETE`. A drift is corrected automatically. Send Claude the `Height check:` line.

## E1. Dataset batch 1 (50 episodes, about 1.5 h)
1. In `scripts\meca500\record_auto.py`, set:
   ```python
   NAME = "basler_stage_touch_dataset_2026-11"
   STAGE = "touch"
   NUM_EPISODES = 50
   RESUME = False
   ```
2. Run it. You don't have to watch after the first few episodes, but stay within reach.
3. Tell Claude when it has finished (and the episode count it printed, if any were discarded).

## Later batches (E2, E3, E4)
For each batch: E0 height check, then the same `NAME` with `RESUME = True` and `NUM_EPISODES = 50`.
Each batch carries on with new targets and paths (the episode seeds follow the episode count).
Claude checks each batch before the next.

## Send Claude
- E0: the `Height check:` line;
- E1: that it finished, and anything odd you saw.

## If something goes wrong
| Symptom | What to do |
|---|---|
| `REFUSING TO START: Focus K-Cube ... not found` | Power the focus K-Cube, plug in its USB, close Kinesis, rerun. |
| `REFUSING TO START: The microscope focus is at ... but the bench was calibrated at ...` | The dish or focus changed since calibration. Don't override it: tell Claude. |
| `Stage controller ... not found` | Close Kinesis; check the BBD302 is on and its USB is plugged in. |
| `Stage channel N will not enable` | Power-cycle the BBD302, wait 10 s, rerun. |
| `The stage is not homed` | After a power cycle: rerun with the arm at home (the script homes it), or home it in Kinesis with the arm at home, then close Kinesis. |
| `STAGE_MOVES needs dish_height_map.json` / `The dish height map ... disagrees` | Run C1 (auto_calibrate with stage 6) first. |
| `... dish would hit the parked arm` or `No stage position ... gave a plan` | Send Claude the output. Nothing moved. |
| The dish slides when the stage moves | Press **Esc** (record_auto) or **q** (auto_calibrate), tape the dish down better, rerun C1. |
| `calibration residual ... exceeds` | The script won't run on a poor calibration. Rerun `auto_calibrate.py`. |
| `CALIBRATION FAILED: The dish moved ... too far to patch` (height check) | The dish has sunk or shifted more than 0.2 mm: set `HEIGHT_CHECK_ONLY = False` and run the full calibration. |
| `... already holds N episodes` | That `NAME` is already in use. Pick a new one. |
| `No collision-free path home from the current pose` | Jog the arm straight up 25–30 mm with `teleoperate_microscope.py`, press **H**, close teleop, rerun. |
| `Timed out waiting for frame from camera` | Check the microscope lamp is on and the pylon Viewer / `capture_frames.py` is closed, then rerun. |
| The arm starts paused, or won't move | Close teleop, the web portal, and anything on the other PC connected to the robot, then rerun. |
| The robot portal or scripts can't reach `192.168.0.100` | The USB-C Ethernet adapter needs `192.168.0.50/24` (see Claude's memory notes); a different adapter needs it set again in an admin PowerShell. |
| `Episode N discarded: ... would collide` | A correction move failed its collision check; the next episode carries on. Send Claude the output if it happens often. |
| The pipette slides at the end of a touch episode | Press **Esc**. Set `touch_offset_mm` to 0.1 in `bench.yaml` and tell Claude. |

## Re-calibrating (after the dish is reseated, the focus changes, or the pipette is knocked)
```powershell
.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py
```
- Close Kinesis first: stage 6 moves the stage.
- **After a pipette change or a knock**, first set `START_ABOVE_DISH_MM = 6.0` in its CONFIG
  block, because the move to the start position is not guarded. Set it back to 4.0 afterwards.
- Watch the first descent: the banner should turn red and the arm should stop the moment the tip
  touches.
- A good result has a pivot residual of about 0.1 mm or less, a scope fit of about 3 px or less,
  verify errors of a few px, and a dish height map residual of a few tens of µm. Send Claude the
  terminal output.
- If the terminal says `contact (torque)` or `contact (force limit)`, the backstop stopped the
  descent instead of the slide detection. Tell Claude.
