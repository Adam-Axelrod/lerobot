# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-04
**Time needed:** about 25 min (D1-D2).

**Where things stand.**
- **C3 (10 touch episodes with stage moves) ran cleanly** on 2026-11-04: no discards, no dish
  sliding.
  - 8 of the 9 measured episodes ended within 50 µm (median 22 µm).
  - The two misses had targets in the dark corners of the image, outside the lit circle. One was
    79 µm; the other couldn't be measured because the tip detector couldn't see the tip there.
  - All six targets inside the lit circle ended within 50 µm.
- **Changed since C3:**
  - targets now stay inside the lit circle (90% of its radius);
  - every touch is offset 15 µm in −y, cancelling a push that repeated in both 10-episode runs;
  - `auto_calibrate.py` has a quick dish-height check (`HEIGHT_CHECK_ONLY`). The dish sits on
    **Blu-tack**, which can creep down slowly.
- Calibration (C1, 2026-11-04): pivot 0.057 mm, scope 1.18 px, dish z 28.55, height-map tilt 0.11°.

## Before you start
- **Close Kinesis.** The scripts drive the stage and read the focus K-Cube themselves; only one
  program can hold the controllers. Both controllers must be on and plugged in by USB.
- **Do not home or move the focus K-Cube**, and don't touch the dish or the Blu-tack: the
  calibration and height map are for the dish exactly where it is now.
- Keep a hand near the e-stop for the first run of each step.

## D1. Dish height check (about 3 min, hands off)
This also measures how far the Blu-tack has crept since C1 this morning.
1. In `scripts\meca500\auto_calibrate.py`, set `HEIGHT_CHECK_ONLY = True`.
2. Run `.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py`.
3. The arm finds the dish, touches the middle of the field three times and goes home. The output ends with:
   - `Height check: ... drift ... um` and `HEIGHT CHECK COMPLETE`;
   - a drift over 25 µm (and under 0.2 mm) is corrected automatically: `shifted dish_height_map.json
     and scope_calibration.json by ...`.
4. **Leave `HEIGHT_CHECK_ONLY = True`:** it is what you will run between dataset batches.
5. Send Claude the output.

## D2. 20-episode touch check (about 20 min)
1. In `scripts\meca500\record_auto.py`, set `NAME = "basler_stage_touch20_2026-MM-DD"` (today's
   date), `STAGE = "touch"`, `NUM_EPISODES = 20`.
2. Run it. Same as C3: stage move, approach, correction pauses, touch.
3. Tell Claude when it has finished, and mention anything odd.

If D2 is within 50 µm on (nearly) every episode, the next step is the 200-episode dataset, recorded
in batches with a D1 height check between them.

## Send Claude
- D1: the terminal output (the drift is the Blu-tack creep since this morning);
- D2: that it finished, and anything odd you saw.

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
