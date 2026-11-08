# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-08 (physical slide and experienced-operator trials done)

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

## Now: no bench work outstanding (operator trials done 2026-11-08)
**Experienced operator (Adam), 20 trials:**
- 12/20 within 50 um, 16/20 within 100 um, median 40 um, 29 s per trial.
- 2 collisions: guard stops by the illuminator in the first 5 s, both from starts with about 5 mm
  of clearance. Nothing touched.
- Run: `human_experienced_20261108-171553`. The 5 practice trials are in `human_practice_*`
  (not scored).

**Guard fixes made during practice:**
- Practice pressed the tip 0.46 mm into the dish, and a stop by the illuminator came 1.7 mm late.
- The SpaceMouse guard now slows the descent near the dish and slows the arm near obstacles
  (`operator_guard.py`). All 20 scored trials ran with the fix.

**Still open, no arm needed:** by-eye checks of the 20 final frames if the paper wants them.

## Earlier today: physical slide done (2026-11-08)
The physical slide ran on 2026-11-08 and the base is back in its screw holes. Nothing on the arm is
outstanding for the ISMR paper (framing A). Still open, no arm needed: Adam's by-eye verdicts on the
`epNNN/final_overlay.png` of the three physical-slide runs (`verdicts.json`, see score_eval.py).

**What happened, for the paper:**
- **The slide.** It was done by hand with the clamps on, and measured 1.32 mm at the microscope:
  d = (-1.00, -0.87) mm.
- **The rotation** reads -0.6 deg but is not resolvable: no-slide calibrations scatter by about 1 deg
  in heading. Only the translation moved the safety model.
- **The pipette** got bent slightly. The tip in the flange frame moved by about 0.3-0.55 mm (pivot
  fits); the ASR tip offset was re-measured as (+1.5, +19.9) px.
- **Incidents, none affecting the data:**
  - the illuminator column was bumped and re-fixed;
  - the focus K-Cube was power-cycled by accident, so its counter restarted at 0 and `focus_mm` is now
    0.0 (same physical focus);
  - the BBD302 needed a power cycle.
- **Results at 1.3 mm** (score_eval, no by-eye verdicts yet):

  | Session | Within 50 um | Median | Run |
  |---|---|---|---|
  | scripted alone | 0/10 | 1324 um | `scripted_20261108-145531` |
  | scripted_asr | 6/10 | 38 um | `scripted_asr_20261108-142822` |
  | policy_asr | 7/10 | 34 um | `basler_stage_touch_decoyrec_2026-11_delta_act_asr_20261108-144107` |

  The policy's approach used the old tip model.

**The base did not return to its old place.** Screwed back into its original holes, it sits
1.49 mm (+0.55, +1.38) from where it was before the slide, at the microscope (hole clearance).
- The bench is recalibrated there (2026-11-08 15:24) and consistent: scripted runs, collection and
  the servo are fine.
- **The learned policies were trained at the old position.** Any new policy run sees about 1.5 mm
  of offset. Re-seat the base and recalibrate, or collect new data (follow-up paper D), before
  comparing a policy with the 0 mm rows.

## You on the SpaceMouse
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
