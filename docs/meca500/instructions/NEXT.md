# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-07 (new plan: learned approach + Huanyu's controller)

## The plan (since 2026-11-06)
The paper's system is now a **learned policy that brings the pipette under the microscope, then hands
over to Huanyu's classical controller** (ActionStateRuntime, ASR). In `scripts\meca500\eval_auto.py`
this is `METHOD = "policy_asr"`. Each trial goes through these stages, all logged:
1. **Approach.** The approach policy (ACT retrained on decoys + recovery episodes) drives while
   Huanyu's tip detector watches every frame.
2. **Handover.** The policy stops the moment the detector sees the tip twice in a row.
3. **Search, only if needed.** If the policy ends with the tip out of view, the arm searches in a spiral
   out to 4 mm from where the policy stopped, 1 mm above the dish. Results are reported with and without it.
4. **Lower.** The tip moves along the pipette to 0.15 mm above the dish.
5. **Servo.** ASR's visual servo moves the tip level onto the ring, stopping within 3 px (40 um).
6. **Touch.** The tip goes straight down onto the dish.

Everything ran end to end without the arm on 2026-11-07: simulated sessions, plus Huanyu's real
detector and servo on real frames. **It has not moved the real arm yet.**

## Now: the bench session, in this order (Adam at the bench; Claude messages before the arm moves)
Every session uses `scripts\meca500\eval_auto.py`. Set its CONFIG block, run
`.venv\Scripts\python.exe scripts\meca500\eval_auto.py`, and send Claude the folder name printed at the
end. All rows use the same targets (`SEED_BASE = 5_000_000`). Do the **E0 height check** before each
session (`auto_calibrate.py` with `HEIGHT_CHECK_ONLY = True`) and close Kinesis.
Loading Huanyu's model adds about a minute at the start of each `*_asr` session.

**Before every session:** `.venv\Scripts\python.exe scripts\meca500\preflight.py` must end with **GO**.
It is read-only, and the arm does not move. It checks the config, the slide state, the calibration
files, the model, Huanyu's detector, the focus, the robot connection, the camera, and that the
detector sees nothing with the arm at home. It printed GO on 2026-11-07.

**Done without the arm (2026-11-07):** a no-motion check of Huanyu's detector on the live feed (arm at
home, 20 frames, no false detections, 0.25 s per frame), and simulated sessions of every stage.

| # | Robot position | Row | Set in CONFIG | Trials | Time |
|---|---|---|---|---|---|
| 1 | 0 mm | **Cautious first trials, hand on the e-stop** | `METHOD = "policy_asr"`, `NUM_EPISODES = 3` | 3 | ~10 min |
| 2 | 0 mm | **Our system** | `METHOD = "policy_asr"`, `NUM_EPISODES = 20` (or `FIRST_EPISODE = 3` to carry on from row 1) | 20 | ~40 min |
| 3 | 0 mm | Scripted + Huanyu's servo | `METHOD = "scripted_asr"`, `NUM_EPISODES = 20` | 20 | ~30 min |
| 4 | 0 mm | Scripted alone | `METHOD = "scripted"`, `NUM_EPISODES = 20` | 20 | ~20 min |
| 5 | 2 mm | Slide, then rows 2, 3, 4 and automatic collection (`METHOD = "expert"`) | `NUM_EPISODES = 10` each | 4 x 10 | ~2 h incl. slide |
| 6 | 4 mm | Same as row 5 | `NUM_EPISODES = 10` each | 4 x 10 | ~2 h incl. slide |
| 7 | 0 mm | Only if time is left: "ACT, no decoys" on the current targets | `METHOD = "policy"`, that model | 20 | ~40 min |

**What to watch in row 1:**
- The arm stops as soon as the tip appears under the microscope.
- A slow move down, then small level moves near the ring, then a slow touch.

**Press Esc or the e-stop if the tip presses or bends, or if a move heads for the dish wall or the
lamp.** If the policy ends with the tip out of view, the arm searches in a spiral 1 mm above the dish:
that is intended.

`BASE_OFFSET_MM` stays `(0.0, 0.0)` for every row here; it is the software slide, for checks only.

## The physical slide (rows 5 and 6)
The robot is slid on the table relative to the microscope. The policy is not told; the safety model
is. Slide **sideways, parallel to the front edge of the microscope**, so the arm is no closer to the
lamp. Use the same direction for 2 and 4 mm.
1. **Just before sliding: E0 height check** (`auto_calibrate.py` with `HEIGHT_CHECK_ONLY = True`).
   The height change the slide measurement reports is then mostly the slide, not hours of dish drift.
   Mark the base's position on the table (tape at two corners).
2. **Slide:** arm at home, power stays on. Loosen the base clamps, slide by 2 mm (then 4 mm from the
   original position) against a ruler or feeler gauge, re-clamp. Note the direction and the ruler reading.
3. **Re-calibrate:** `auto_calibrate.py` with `HEIGHT_CHECK_ONLY = False` and `START_ABOVE_DISH_MM = 6.0`.
   It finds the tip with its own spiral search and re-measures the scope calibration and the dish
   height map. It saves the pre-slide calibration as the newest `scope_calibration.*.bak.json`. Set
   `START_ABOVE_DISH_MM` back to 4.0 afterwards.
4. **Measure the slide and move the safety model:**
   `.venv\Scripts\python.exe scripts\meca500\apply_slide.py --before <that newest scope_calibration .bak.json>`.
   It prints the measured slide (x, y in mm), its rotation, and the dish height change (with how many
   minutes passed between the two calibrations). It moves every obstacle by the slide. Send Claude the
   output; it goes in the paper with your ruler reading.
   **The methods keep the pre-slide in-plane calibration.** apply_slide pins it as
   `scope_calibration.pinned.json`. Every method aims with its pixel <-> XY map: the policy,
   scripted, scripted + servo and the expert. The dish **height** comes from the current calibration
   and dish map, as after every session's E0 check: the slide tests in-plane registration. The new
   in-plane calibration is used only for the measured slide, the moved obstacles and the final FK
   measurement. `eval_auto.py` and `preflight.py` refuse to start if the slide files don't match.
   Before each later session at this position, the E0 height check runs as usual.
5. Run the sessions of row 5 (or 6).
6. **Before the next slide or when done:** slide back to the tape marks,
   `apply_slide.py --reset`, and recalibrate as in step 3.

## You on the SpaceMouse (not in the current order; only if time is left)
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
