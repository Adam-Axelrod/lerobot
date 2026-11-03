# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-03
**Time needed:** about 40 min (C1-C3).

**Where things stand.**
- **The dish sits on an acrylic holder** in the MLS203 stage, refocused (focus K-Cube at
  **4.000 mm**), and the arm model and calibration match it (2026-11-03): tip pivot 0.076 mm, scope
  1.26 px, dish z 26.83.
- **Vision correction works:** 10 of 10 touches ended within 50 µm of the target (median 12 µm,
  worst 45). The arm pauses on the final approach, finds the tip in the image and corrects.
- **New, not yet tried on the arm:**
  - random dish-stage moves between episodes (`record_auto.py`, `STAGE_MOVES = True`), so the
    policy sees a different patch of dish each time;
  - a dish-height map, the new 6th stage of `auto_calibrate.py`, so each episode touches at the
    right height for its stage position;
  - an extra correction checkpoint 1 mm out when the first one can't see the tip.
  All of it passed the offline tests and the simulator.
- **Dropped for now:** the zirconia beads. The dataset uses the digital target over the empty dish.

## Before you start
- **Fix the dish to the acrylic,** with a few small pieces of double-sided tape under its rim.
  From now on the stage moves the dish between episodes; it moves gently, but a loose dish could
  still creep, and every creep shifts the dish under the calibration.
- **Close Kinesis.** The scripts drive the stage and read the focus K-Cube themselves; only one
  program can hold the controllers. Both controllers must be on and plugged in by USB.
- **Do not home or move the focus K-Cube.** It must stay at 4.000 mm (the scripts refuse to
  start otherwise).
- Keep a hand near the e-stop for the first run of each step.

## C1. Re-calibrate with the dish height map (about 20 min, hands off)
The dish was taped down, so it may have shifted slightly: recalibrate, now with the new stage 6.
```powershell
.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py
```
- Stages 1-5 run as before.
- **Stage 6 is new:** the arm backs the tip out of view and lifts it, the stage moves the dish to
  a new position, then the tip comes back and touches the dish. This repeats for 10 stage
  positions, and the stage returns to (55, 37.5) at the end.
- Watch the first two or three stage moves: the dish must ride with the stage, not slide, and
  nothing may come near the pipette.
- A good result ends with `dish height map: tilt ... deg, residual ... um rms`, with a residual of
  a few tens of µm or less. Up to two touches may be dropped as outliers.
- Send Claude the terminal output.

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
- C1: the `auto_calibrate` terminal output;
- C2/C3: that they finished, and anything odd you saw.

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
  block, because the move to the start position is not guarded. Set it back to 2.0 afterwards.
- Watch the first descent: the banner should turn red and the arm should stop the moment the tip
  touches.
- A good result has a pivot residual of about 0.1 mm or less, a scope fit of about 3 px or less,
  verify errors of a few px, and a dish height map residual of a few tens of µm. Send Claude the
  terminal output.
- If the terminal says `contact (torque)` or `contact (force limit)`, the backstop stopped the
  descent instead of the slide detection. Tell Claude.
