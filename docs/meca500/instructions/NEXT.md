# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-05 (paper order)

## Now: record the decoy dataset (Claude is running it)
**The first ACT test failed: 0 of 20 within 50 um, median 2.0 mm** (2026-11-05,
`outputs\eval_auto\basler_stage_touch_2026-11_delta_act_20261105-121850`). That is the paper's
"ACT, no decoys" row; it ran on the fixed rollout code, so it does not need rerunning. Probes show why
it failed: the expert aimed every path at the target, so the joint state at the pre-touch point gave
the target away; the policy read the marker to set off but not to finish. `record_auto.py` now plans each path
to a DECOY up to 1.6 mm from the target while the marker stays on the true target; the pre-touch
correction then closes the gap in the recorded final approach, so only the image says where to go.

1. **Batch 1** (`basler_stage_touch_decoy_2026-11`, 50 episodes) is recording from 15:20 on 2026-11-05.
   While watching: each episode pauses two or three times near the dish while the tip is detected;
   after the first pause the tip moves sideways by up to 1.6 mm before descending. Both are intended.
   A pause followed by the arm leaving without touching means an episode was discarded; the next
   attempt uses a new target. **To stop early press Esc** - never close the window or kill the
   process: the dataset is only finalised on a clean exit.
2. **Batch 2:** E0 height check, then `RESUME = True` and `record_auto.py` again (about 45 min).
3. Claude converts to deltas and trains **ACT model A**: the same settings as the first ACT, so the
   comparison isolates the decoys. A separately labelled model B (precision-step loss weight and
   joint-state dropout) is optional; Claude will ask first.

## Next: the paper's results table (ISMR deadline 10 Nov; last bench day Sun 8 Nov)
Work down this list in order and stop when time runs out. All of it uses `scripts\meca500\eval_auto.py`:
set the values in its CONFIG block, run it, and send Claude the folder name it prints. Do the **E0
height check** before each session. Every run uses the same seeds (`SEED_BASE = 5_000_000`), so every
method gets the same targets. **If a recalibration is needed, tell Claude:** it can change the test
set, and `score_eval` refuses mismatched targets.

**Do not use the older Diffusion, ACT-50 or ACT-100 models for the table:** they were trained on the
non-decoy dataset and have the same shortcut.

| # | Row | Set in CONFIG | Time |
|---|---|---|---|
| 1 | **Go/no-go: ACT on decoys (model A)** | `METHOD = "policy"`, `IMAGE_SHIFT_PX = 0`, `MODEL` and `EVAL_NAME` for model A | ~40 min |
| 2 | Automatic collection, from its logs | Nothing to run on the bench: see below | - |
| 3 | Scripted controller | `METHOD = "scripted"` | ~40 min |
| 4a | ACT on decoys, small knock | `METHOD = "policy"`, `IMAGE_SHIFT_PX = 16` | ~40 min |
| 4b | ACT on decoys, big knock | `METHOD = "policy"`, `IMAGE_SHIFT_PX = 58` | ~40 min |
| 4c | Scripted, both knocks | `METHOD = "scripted"`, `NUM_EPISODES = 10`, `IMAGE_SHIFT_PX` 16, then 58 | ~40 min |
| 4d | Automatic collection, both knocks | `METHOD = "expert"`, `NUM_EPISODES = 10`, `IMAGE_SHIFT_PX` 16, then 58 | ~40 min |
| 5 | You on the SpaceMouse | `METHOD = "human"`, `OPERATOR = "experienced"` (start with 2-3 trials) | ~1 h |
| 6 | Diffusion, retrained on decoys | `METHOD = "policy"`, `MODEL` and `EVAL_NAME` for that model | ~45 min |
| 7 | ACT on 50 decoy demos (data scaling; 200 only if 100 looks close) | `METHOD = "policy"`, `MODEL` and `EVAL_NAME` | ~45 min |
| 8, 9 | Other operators, only if time allows | `METHOD = "human"`, `OPERATOR = "embryology"`, `"novice1"`, ... (at least 3 novices) | ~1 h each |

**After row 1, stop if ACT misses most targets** and tell Claude before running anything else. Claude
also reruns the ring-moving probe on model A to check it now reads the image.
Skipped unless something changes: ACT with precision off (only if `clamped_steps` shows the flag
matters), and Huanyu's visual servoing (only if he offers).

Set `IMAGE_SHIFT_PX` back to 0 after the knock rows. A session that stops early can carry on: set
`FIRST_EPISODE` to the next trial number.

**Row 2.** On the PC that has the dataset:
`.venv\Scripts\python.exe scripts\meca500\score_eval.py --dataset <path to basler_stage_touch_decoy_2026-11>`.
Discarded attempts count as failures: batch 2 onwards logs them in the dataset's
`autocollect_discards.jsonl`; batch 1's come from `outputs\record_decoy_batch1b.log`. Send Claude the line.

**Human trials (first time: try 2-3 trials with `NUM_EPISODES = 3` and a hand on the e-stop).** This
mode has only been tested without the arm.
- Plug in the SpaceMouse. The arm moves to each start by itself, then a window shows the microscope
  feed and the ring. Nothing moves until the right arrow starts the trial; the clock starts then.
- **Translation only:** twisting the SpaceMouse does nothing, and the pipette keeps its angle.
- **Coming in:** travel along the pipette's own direction, down the gap by the lamp. **Enter** switches
  between coarse and fine.
- **Finishing:** put the tip in the ring and lower it until the title says `ON THE DISH`: it cannot go
  lower. Press the right arrow when you are happy. The limit is 120 s.
- Going too close to the lamp or the dish wall ends the trial and counts as a collision. If the next
  move cannot be planned, the title asks you to back the pipette out and press the right arrow.
- Give each new operator the same few minutes of practice first, using `NUM_EPISODES = 3` and a
  different `OPERATOR` name so the practice runs are not scored as trials.

| Symptom | What to do |
|---|---|
| `Failed to open SpaceMouse` | Plug it in, close 3Dconnexion's own software and anything else using it, rerun. |
| The tip stops before `ON THE DISH`, and the title never changes | Stop and tell Claude: the dish height may have drifted. |
| The tip visibly presses or bends | Press **Esc** or the e-stop, and tell Claude. |

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
- **Training:** ACT (200 demos) is done and tested (0/20, the "no decoys" row). Diffusion and ACT on
  50/100 demos were trained on this dataset too, but they share the shortcut: not for the table.

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
