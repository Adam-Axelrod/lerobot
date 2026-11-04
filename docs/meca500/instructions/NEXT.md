# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-04
**Time needed:** about 20 min (C2-C3).

**Where things stand.**
- **C1 is done (2026-11-04).**
  - The re-taped dish sits **1.63 mm higher** than before, at dish z 28.55.
  - Tip pivot fit: 0.057 mm.
  - Dish height map: tilt 0.11°, residual 14 µm. Height changes by at most 24 µm across the
    random-move area.
- **The scope fit needed one fix.** The first grid touch used the old dish height, started 0.6 mm
  inside the dish and was 0.58 mm off. Claude refitted the calibration without it, from the saved
  touches: 1.18 px rms (16 µm), agreeing with the height map to 6 µm. `auto_calibrate.py` now uses
  the measured height, drops bad touches itself, and starts 4 mm up instead of 2. No need to rerun C1.
- **Vision correction works:** 10 of 10 touches ended within 50 µm of the target (median 12 µm,
  worst 45). The arm pauses on the final approach, finds the tip in the image and corrects.
- **New, not yet tried in `record_auto`:** random dish-stage moves between episodes
  (`STAGE_MOVES = True`), touching at the mapped height for each stage position, and an extra
  correction checkpoint 1 mm out when the first one can't see the tip.
- **Dropped for now:** the zirconia beads. The dataset uses the digital target over the empty dish.

## Before you start
- **Close Kinesis.** The scripts drive the stage and read the focus K-Cube themselves; only one
  program can hold the controllers. Both controllers must be on and plugged in by USB.
- **Do not home or move the focus K-Cube**, and don't touch the dish or its tape: the calibration
  and height map are for the dish exactly where it is now.
- Keep a hand near the e-stop for the first run of each step.

## C2. Hover check with stage moves (3 episodes, about 5 min)
1. In `scripts\meca500\record_auto.py`, set:
   ```python
   NAME = "basler_stage_hover_2026-MM-DD"   # today's date
   STAGE = "hover"
   NUM_EPISODES = 3
   ```
   (`STAGE_MOVES = True` and `VISION_CORRECTION = True` are already set.)
2. Run `.venv\Scripts\python.exe scripts\meca500\record_auto.py`.
3. Before each episode, while the arm waits at its start pose, the stage moves the dish (the
   terminal says `Moving the dish to stage [...]`). Then the episode runs; the tip stops 5 mm
   above the marker. The dish background should look different each time.

## C3. Touch test with stage moves (10 episodes, about 10 min)
1. Set `NAME = "basler_stage_touch10_2026-MM-DD"`, `STAGE = "touch"`, `NUM_EPISODES = 10`.
2. Run it again. Each episode: stage move, approach, two short correction pauses, touch.
3. Tell Claude when it has finished. Claude reads the final errors and stage positions from the
   dataset. Mention anything odd: the dish sliding, a slide or bend at a touch, discarded
   episodes.

If C3 looks like B5 (within 50 µm, no slides), the next step is the real 200-episode dataset.

## Send Claude
- C2/C3: that they finished, and anything odd you saw (dish sliding, discarded episodes).

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
