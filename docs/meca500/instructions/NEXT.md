# Next at the bench

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Last updated:** 2026-11-08 (one physical 2 mm slide today)

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

## Now: the physical 2 mm slide (Sun 8 Nov, the last bench day)
Decided 2026-11-08: the paper is framing A for ISMR, and this run removes its "software slide"
caveat at 2 mm. Seeds 0-9 (`SEED_BASE = 5_000_000`), as in the software slides. 4 mm is not
repeated physically.

**Who does what:** you do everything physical and keep a hand on the e-stop. Claude sets every
CONFIG block and runs every script on this PC, and tells you before each arm motion. Answer
Claude's "ready?" only when the step before it is done.

**Press the e-stop (or Esc in the script's window) if:**
- the tip presses or bends;
- a move heads for the dish wall or the lamp;
- anything is loose after re-clamping.

A spiral search 1 mm above the dish is intended.

### Steps
1. **Set up (you).**
   - Close Kinesis and the pylon Viewer.
   - The microscope lamp is on and the arm is at home.
   - The focus K-Cube is powered. Don't touch it.
2. **Height check (Claude, about 3 min, the arm moves).**
   - `auto_calibrate.py` with `HEIGHT_CHECK_ONLY = True` runs. It should end with `HEIGHT CHECK COMPLETE`.
   - Its calibration is the "before" for the slide measurement, so nothing may move between this
     step and step 4 except the robot base.
3. **Mark the base (you).** Put tape at two corners of the robot base on the table, and a ruler or
   feeler gauge against the base's side that faces the slide direction.
4. **Slide (you).**
   - The arm stays at home and the power stays on.
   - Loosen the base clamps.
   - Slide the base **2 mm towards robot +y**. Standing behind the robot (cable side) facing along
     robot +x, the microscope's optical axis is about 238 mm ahead and 85 mm to your right; +y is to
     your **left**. It is also parallel to the microscope's front edge: no closer to the lamp.
   - Keep it straight: don't twist the base.
   - Re-clamp firmly.
   - Tell Claude the ruler reading and the direction.
5. **Recalibrate (Claude, the arm moves).**
   - `auto_calibrate.py` runs in full mode (`HEIGHT_CHECK_ONLY = False`, `START_ABOVE_DISH_MM = 6.0`).
     It finds the tip with its own spiral search, then re-measures the scope calibration and the
     dish height map.
   - Watch the first descent; the tip starts 6 mm above the dish.
6. **Measure the slide (Claude, no motion).**
   - `apply_slide.py --before <the step 2 calibration>` prints the measured slide (x, y), its
     rotation and the height change, and moves the safety model by it.
   - Expect about (0, +2) mm and a rotation under 0.5 deg. If it reports a rotation over 0.5 deg,
     re-seat the base squarely (Claude says so) and repeat steps 4-6.
   - Methods keep the pre-slide pixel <-> XY map (`scope_calibration.pinned.json`); the policy is
     told nothing.
7. **Preflight (Claude, no motion).** `preflight.py` must end with **GO**.
8. **Trials (Claude runs them, you watch with a hand on the e-stop).** The height check runs before
   each session.

   | # | Session | Trials | Time |
   |---|---|---|---|
   | a | Scripted + Huanyu's servo (`scripted_asr`) | 10 | ~15 min |
   | b | Our system (`policy_asr`, the deployed decoyrec ACT) | 10 | ~25 min |
   | c | Only if time is left: scripted alone (`scripted`); it should miss by about 2 mm | 10 | ~10 min |
9. **Slide back (you, then Claude).**
   - Arm at home. You loosen the clamps, slide back to the tape marks and re-clamp.
   - Claude runs `apply_slide.py --reset`, then the full recalibration as in step 5, then sets
     `START_ABOVE_DISH_MM` back to 4.0.

`BASE_OFFSET_MM` stays `(0.0, 0.0)`. That is the software slide; it is refused on top of a physical one.
`eval_auto.py` and `preflight.py` refuse to start if the slide files don't agree with each other.

### Done without the arm for this (2026-11-08)
- On a copy of the bench files in a synthetic +2 mm post-slide state: all 10 seeds plan
  collision-free for both methods, and so does the transit from home.
- At -2 mm the scripted plans also pass, so a slide the wrong way is still safe.
- `apply_slide.py` measured the synthetic slide correctly.

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
