# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-05

## Now: test the ACT policy (about 45 min)
The 200-demo ACT model is on this PC. `eval_auto.py` sets up each episode exactly as recording
did (stage position, start pose, target) with targets the model has never seen, then the policy
drives. Every command is checked before it is sent: step clamp, collision model, and the tip may
not push more than 0.1 mm into the dish.

**Nothing else may use the GPU during the test** (training slows the policy below 20 Hz).

1. **E0 height check (about 3 min):** `HEIGHT_CHECK_ONLY = True` is already set. Close Kinesis, then run
   `.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py`. It should end with `HEIGHT CHECK COMPLETE`.
2. **Policy test (20 episodes, about 30-40 min):**
   `.venv\Scripts\python.exe scripts\meca500\eval_auto.py`
   - **Watch the first few episodes with a hand on the e-stop.** The policy has never driven the arm.
   - **Esc** stops the session (the arm backs out and goes home). **Right arrow** ends the current
     episode early; it is still measured.
   - An episode ending `stopped: ...` is a safety check doing its job: the next episode carries on.
3. Send Claude the summary line at the end, and the folder name it prints (`outputs\eval_auto\...`).

| Symptom | What to do |
|---|---|
| `Control loop ran at N Hz, below the 20 Hz ...` | Something else is using the GPU. Finish the session, tell Claude. |
| The pipette slides or presses on the dish | Press **Esc** and tell Claude. |
| The arm heads somewhere clearly wrong | Press **Esc** (or the e-stop if it is close to anything). |

## Collection (complete)
`basler_stage_touch_dataset_2026-11` holds 206 episodes, 200 of them usable.

| Batch | Episodes | On target (checked by eye on the raw frames) |
|---|---|---|
| E1 | 0-49 | 44 of 50 |
| E2 | 50-99 | 50 of 50 |
| E3 | 100-149 | 50 of 50 |
| E4 | 150-205 | 56 of 56 (median 15 um, max 44 um) |

- **Exclusions:** E1 episodes 9, 10, 21, 34, 35 and 44 are left out at training time. They are listed
  in the dataset's `autocollect/excluded_episodes.json`.
- **Dish drift:** the height checks drifted +3, +28, -22, -7 and -43 um over the day (Blu-tack).
  Each was corrected before its batch.
- **Training:** ACT (200 demos) is done. Diffusion finishes on CREATE about 22:00 on 2026-11-05.
  ACT on 50 and 100 demos (the data-scaling curve) is training on this PC.

## If you record more later
1. **E0 height check (about 3 min):** `HEIGHT_CHECK_ONLY = True` is already set. Run
   `.venv\Scripts\python.exe scripts\meca500\auto_calibrate.py`; it should end with
   `HEIGHT CHECK COMPLETE`.
2. **Batch:** `scripts\meca500\record_auto.py` with the same `NAME` and `RESUME = True`.
   A 50-episode batch takes about 45 min.
- **Close Kinesis first, and do not move the focus K-Cube.**
- The large dark feature in the dish is a scratch; leave it.

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
