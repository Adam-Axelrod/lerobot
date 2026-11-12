# Why the learned approach does not reach 50 µm

**Last updated:** 2026-11-12, evening. Evidence comes from four sources:
- bench runs on 5–8 Nov at the 0 mm base position;
- the offline perception test of 10 Nov, which swapped theories 1 and 2;
- the finish-only batch and the first finish-policy bench trials, on 11 Nov (theory 1);
- the finish-policy bench session, the fixed-point probe and the learned refinement test, on 12 Nov.

## Where we are (12 Nov)

**Results so far.** All finish results below are from an in-view start, 0.1–1.5 mm beside the ring,
on the same 10 targets; vision is the detected tip against the target pixel, 13.4 µm/px.

| System | Within 50 µm (vision) | Median | Notes |
|---|---|---|---|
| End-to-end ACT model A, from the pre-touch point | 0/5 | 420 µm (FK) | 6 Nov |
| ACT finish policy, crop only, scripted descent | 5/10 | 51 µm | run (a) |
| ACT finish policy, crop + joints, scripted descent | 7/10 | 38 µm | run (b) |
| ACT finish policy + learned tip localiser at a 50 µm hover | 10/10 | 22 µm (max 39) | the theory 1 test below; aimed the glass end |
| **The same, aiming the pipette opening ("the tip" since 12 Nov)** | **10/10** | **29 µm** (max 45), measured to the opening | glass-end runs above score 0–1/10 against the opening |
| Scripted teacher (finish batch) | 194/200 | 14 µm | 11 Nov |

**The two failures, and where each stands.**
- **The finish, from in view, is close to solved.** The ACT finish policy gets the tip onto the ring
  but settles where its own implicit tip estimate is zero, which is 1–8 px off. A separate learned
  regressor of the tip-to-ring offset reads the same frames to about 1 px, and using it for the last
  correction gives 10/10 within 50 µm (theory 1, "Fixed-point probe" and "Refinement test").
- **Getting into the finish's range is unsolved.** From the scripted pre-touch point the tip starts
  1.7–4.2 mm off in 8/10 trials, outside the crop and the demos, and the finish policy does not
  recover (runs c and d). The end-to-end approach still arrives 2–3 mm off (theory 4).

**Status of each theory.**

| Theory | Status after 12 Nov |
|---|---|
| 1. Training never rewards the correction | **Supported, refined.** Finish-only data raised ring-following; the residual is an imprecise implicit tip estimate, fixed by an explicit one |
| 2. The full frame is too coarse | Weakened. The crop helps; the frames are 8.6× downsampled from the sensor, and the optics resolve about 2× finer than our pixels |
| 3. The joint state is a shortcut | Not supported in range (joints 7/10 vs 5/10 without); out of range the joint model barely moves |
| 4. The approach is open-loop and random | Open. Now the main obstacle for the full system |
| 5. Compounding error off the demos | Open; waits on 4 |
| 6. Images cannot show height | **Supported.** Policy-driven height pressed up to 99 µm into the dish; scripted height touches every time |

**Is 50 µm a fair bar at this magnification?** (analysis 12 Nov)
- In the frames everything uses, 50 µm is **3.7 px**. The pipette end is about 30 px (400 µm) across,
  so 50 µm is an eighth of its width, and "on target" depends on which point of the blunt end counts.
- The detector repeats to about 1 px (13 µm); the teacher ends a median 1.07 px from the target; the
  learned localiser reaches 13 µm held out. So 3.7 px is **measurable** in the image.
- It is **not measurable from the kinematics**: the scope calibration's residual is 2.1 px rms
  (28 µm), so FK errors below about 50 µm are mostly calibration scatter. Score in the image.
- The camera has headroom. The sensor is 5496×3672 at 1.6 µm/px, binned 2× and then downscaled to
  427×640, 8.6× in all. A native-resolution frame shows the optics resolve features of about
  10–20 µm, so a crop at about 6–7 µm/px would keep nearly all the optical detail and put 50 µm at
  7–8 px.
- **By-eye standard (Adam, 12 Nov).** 53 of 54 blind final frames judged on the review page
  `https://claude.ai/artifact/1M4UQXEG5zhKnsKKb2uHge` (key `outputs/finish_review/review_key_20261112.json`,
  verdicts `outputs/finish_review/verdicts_20261112.csv`): 24 success, 22 borderline, 7 fail.
  - **The detector's apex distance does not predict the verdicts** (AUC success vs fail 0.56,
    success vs borderline 0.39). Successes include 74–115 µm by the detector; fails include 22–43 µm.
  - **What does: whether the pipette end covers the target.** All 7 fails have the apex short of
    the target along the pipette (−0.6 to −8.3 px); successes mostly have it past (median +1.9 px).
  - Distance from a point **5–6 px (70–80 µm) behind the apex along the pipette** separates success
    from fail with AUC 0.99 (0.85 against borderline); the fit is flat from 5 to 8 px. On the
    frames that point is the bright lumen opening at the end of the pipette; the detector's apex is
    the outer edge of the glass.
  - By this standard the scripted teacher is 4/9 success, 5 borderline: it puts the outer apex on
    the target, which reads as borderline. The refinement run is the same, 5 success, 5 borderline,
    0 fail. Everything so far (teacher labels, policy, localiser, scoring) aims the wrong point.
  - **Consequence:** redefine the aim point as the lumen opening, about 75 µm behind the apex, in
    the teacher, the localiser's target and the scoring; then 50 µm from the opening is the bar.
- **"The tip" is now the centre of the pipette opening (Adam, 12 Nov).**
  - **Found in each frame** by `tip_detect.find_opening`: along the pipette axis behind the glass
    end, the end wall is a dark minimum and the opening the bright peak after it. Seen in 224/232
    test frames, else a fixed 7 px fallback. Against the verdicts, opening-to-target distance gives
    AUC 1.00 success vs fail and 0.91 vs borderline; every frame within 61 µm was judged success,
    the first borderline is at 63 µm and the first fail at 97 µm.
  - **Where it applies:** `eval_auto` scoring (`vision_tip_px` / `vision_error_um` are the opening;
    the glass end is kept as `vision_apex_*`), the teacher's final error (`record_auto`), the
    localiser's labels (`train_tip_localiser.py`), and the aim of every plan and of ASR through
    `bench.yaml` `target.opening_offset_px` = (−0.66, +5.48) px, opening minus glass end.
  - **Where it does not:** contact detection, the calibration touches, the tool model and the
    teacher's checkpoint corrections keep the glass end, which is what touches and what the
    calibration predicts. So FK error now reads about 75 µm higher by design.
  - **Old runs re-scored** with `scripts/meca500/rescore_opening.py` (`results_opening.jsonl` per
    run). Opening medians: run (a) 85 µm (0/10), run (b) 93 µm (1/10), the first refinement run
    68 µm (1/10). Every earlier system aimed the glass end.
  - **Bench, refinement aiming the opening** (localiser `outputs/tip_localiser/20261112-200102`,
    retrained on opening labels: 13 µm held out; same 10 targets;
    `outputs/eval_auto/finish_2026-11_crop256_img_act_finish_zscripted_refine_n15_20261112-200658`):
    **10/10 within 50 µm of the opening, median 29 µm, max 45 µm**; opening found in 10/10 frames.
    Every result is inside Adam's by-eye success zone (≤ 61 µm).
  - **What is left is the touchdown slide:** from the last hover estimate to the final frame the
    opening moves (+0.5, −2.0) px on average, sd about 1 px, forward along the pipette (27 µm).
    Aiming the hover correction 2 px behind the target should take the median to about 15 µm.

## Background: what the end-to-end policies did (5–8 Nov)

Every learned policy we trained on the scripted demos ends **millimetres** from the target, not tens
of micrometres. It gets there only when ASR's visual servo does the finish.

| Model (ACT unless noted) | Bench result | Run |
|---|---|---|
| No decoys: paths aimed at the target, joints + image | 0/20 within 50 µm, median ~2 mm | `basler_stage_touch_2026-11_delta_act_20261105-*` |
| A: decoy paths, joints + image | 0/14, median 2769 µm, tip seen in 57% | `..._decoy_2026-11_delta_act_20261105-220640` |
| A, re-planning every 10 steps | 0/10, median 2490 µm (kinematics) | `..._decoy_..._coarse10_20261106-140621` |
| A, finishing from the scripted pre-touch point | median 420 µm (65–1547), 2 of 5 hovering | `..._decoy_..._handover_20261106-143002` |
| E: image only, tip actions, ring loss | 0/5, all stopped 12–24 mm short of the dish | `..._tip_delta_ring1_act_20261106-130522` |
| E, finishing from the scripted pre-touch point | 48–88 µm across in 4/5, but hovering 1.1–1.5 mm up | `..._tip_delta_ring1_act_handover_20261106-144053` |
| decoyrec: decoys + recovery, approach only, then ASR | 12/20 needed the search to find the tip | `..._decoyrec_..._asr_20261107-143233` |

**Measurements behind the theories:**

- **Marker-shift probe** (`probe_marker_shift.py`, `outputs/probe_marker_shift_*.log`).
  - How it works: the target marker is moved 1.07 mm in a recorded frame, and the gain is how much
    of that move the predicted action chunk follows (1 = all of it, 0 = none).
  - Gains at pre-touch:

    | Model | Gain |
    |---|---|
    | No decoys | 0.007 |
    | A | 0.015 |
    | C, D (image only) | 0.11–0.12 |
    | E, F (ring loss) | 0.015–0.019 |

  - Gains at the "near" checkpoint are 0.07–0.31 for every model.
  - Changing only the background texture moves the prediction about as much as moving the marker,
    and usually more.
- **Where the error arises** (`bench_error_timeline.py`, `outputs/bench_error_timeline*.csv`).
  - Median distance of the aimed landing point from the target, at the moment the precision phase
    starts:

    | | At precision entry |
    |---|---|
    | Expert, no-decoy dataset | 179 µm |
    | Expert, decoy dataset (aimed at the decoy) | 669 µm |
    | No-decoy model | 2066 µm |
    | Model A | 2701 µm |
    | Model A, re-planning every 10 steps | 2374 µm |

  - "Always aim at the average target" would already score 1533 µm.
- **Leak regression** (`leak_regression.py`, re-run today). This is a linear fit of the target
  position from the joint state at the pre-touch point, cross-validated.
  - No-decoy dataset: 219–235 µm RMS, against 1533 µm for the average-target baseline.
  - Decoy dataset: 835–879 µm RMS, against 1519 µm.
  - So the joints still give away about half of the target's position even with decoys.
- **Teacher-forced coarse phase** (`outputs/probe_shift_causes_modelA.log`), on 20 decoy episodes.
  - Model A's predicted chunks differ from the recorded ones by SD 1.7–2.1 mm per chunk.
  - Summed over an episode, the difference is SD 4.8–5.5 mm.
- **Things that did not help:**
  - joint-state dropout 0.3;
  - 10× loss weight on precision steps;
  - the ring-localisation auxiliary loss (E, F);
  - shift augmentation and ResNet50 with dilation (D);
  - recovery episodes (decoyrec);
  - re-planning every 10 steps;
  - joint-state slide augmentation. Fine-tuned with slides up to 5 mm, the model scored 2/10 with the
    base moved 4 mm, against 4/10 without the augmentation.

There are two separate failures:

- **The finish.** From a good pre-touch point, the best joint-state model ends at a median of
  420 µm.
- **The approach.** On its own, the policy reaches the precision phase 2–3 mm off.

Theories 1–3 are about the finish. Theories 4–5 are about the approach.

---

## The theories, most likely first

### 1. Nothing in training rewards the correction, so the decoder ignores the image

**Status (12 Nov): supported, and narrowed.** Finish-only data made the policy follow the ring; what
is left is that its implicit estimate of the tip-to-ring offset is only good to 1–8 px, and the
policy stops where that estimate is zero. An explicit learned estimate fixes it on the bench
(10/10 within 50 µm). The sections below are in the order the evidence came in.

**Theory.**
- The only steps where the image is needed (the decoy correction and the last ~0.2 mm) are a few
  percent of the 47k frames.
- They are also about 20× smaller moves than the coarse steps.
- The training loss is dominated by the coarse path. Copying the average path costs almost nothing,
  so the gradient never pushes the decoder to read the image.

**For.**
- The probe gain is near zero exactly at pre-touch, where the correction happens.
- A 10× precision weight did not move it.
- Each decoy episode gives one correction, so there are about 100 distinct correction examples in
  total.

- **Since 2026-11-10:** the policy's frozen encoder locates the tip relative to the ring to a median
  of 69 µm at near (theory 2's test). The information reaches the encoder, and the actions ignore it.

**Against.**
- The 10× precision weight was one setting on the same data. A stronger signal, many more
  correction examples, has not been tried.

**Solution.**
- Collect many short finish-only episodes. Each starts with the tip in view, 0.1–1.5 mm from a random
  target, and records only the correction. That gives thousands a day, unattended.
- Train the finish policy only on those, with the crop from theory 2.

**Experiment.** The script is `scripts/meca500/finish_policy_offline.py`, about 1.5–2 h of GPU.
- Train a finish-only behaviour-cloning policy on the precision phase of the 200 decoy and recovery
  episodes, held out by episode:
  - no joint state;
  - tip motion along the image axes;
  - ImageNet ResNet18 on a 192 px crop at the target, or on the full frame.
- At the two correction holds, measure:
  - the marker gain (ring moved 266 µm on the raw frame);
  - how well it tracks each episode's own recorded move.
- One recorded chunk covers about 94% of the remaining path at near, so a ring-follower should show
  a gain near 0.9 there.
- Probe it with `probe_marker_shift.py`. The prediction is a pre-touch gain above 0.8, against
  ≤ 0.12 today.
**Result (2026-11-10, `outputs/finish_policy_offline/20261110-161238/`, held out by episode).**

| At the near hold | Crop finish policy | Full-frame finish policy |
|---|---|---|
| Marker gain, ring moved 266 µm (ceiling 0.98) | 0.26 [IQR 0.24, 0.30] | 0.19 [0.14, 0.24] |
| Tracking of each episode's own move (1 = exact) | 0.60 | 0.19 |
| Chunk error at the holds (zero baseline 108 µm) | 78 µm | 169 µm |

- **Training on the finish alone did not make it follow the ring.**
  - The crop policy follows a quarter of a 266 µm ring move.
  - That is barely more than ACT model A's 0.20 at near.
  - The full-frame policy matches ACT.
- **The crop helps.** Tracking is 0.60 against 0.19, and the chunk error is half.
- **The data never shows a correction that size near the dish.**
  - The recorded corrections at the near hold are a median of 31 µm, at most 105 µm.
  - At pre-touch they are a median of 36 µm (p90 76 µm).
  - So a 266 µm ring move at near asks the policy to extrapolate. The demos contain almost no
    examples of "the tip is 100–300 µm beside the ring, move onto it".
- **Pre-touch is not informative.** One recorded chunk covers only 6% of the remaining path there,
  so the gain ceiling is 0.06.
- **What it means:** the remaining reason is the data, not only the loss weighting.
  - The existing demos almost never show the move the finish needs.
  - That makes the finish-only batch below the next step: tip in view, 0.1–1.5 mm from a random
    target.

**Follow-up (2026-11-11, `outputs/finish_policy_offline/20261111-080937_shift3/`, crop only).**
- Same test with the ring moved 40 µm (3 px), which is inside the recorded corrections.
- Marker gain at near: **−0.01 [IQR −0.57, 0.16]**. Tracking 0.61 and chunk error 82 µm are as
  before.
- So the crop policy does not follow the ring even within the range the demos cover.
- It predicts each episode's own move fairly well (tracking 0.61), but moving the ring does not change
  the prediction. It reads the move from something else in the crop: the tip's phase or motion, the
  dish texture. It does not read the tip-to-ring offset.
- **What it means:**
  - More data of the same kind is unlikely to be enough on its own.
  - The finish-only batch should come with a training change that forces the offset in:
    - an auxiliary loss predicting the tip-to-ring offset (the probe shows the encoder can, 17 µm);
    - or the offset as an explicit input;
    - or ring-position augmentation on recorded frames, shifting the ring and the action label
      together.
  - The augmentation can be tried offline on the existing data first.

**Finish-only batch (recorded 2026-11-11, `basler_stage_touch_finish_2026-11`, 200 episodes).**
- `record_auto.py` with `RECOVERY_KIND = "finish"` (`planner.plan_finish`).
  - The tip starts in view, 0.1–1.5 mm beside the ring (uniform direction) and 0.5–1.5 mm above
    the touch.
  - The recording is the move onto the ring and the touch, about 125 frames each.
- 3 h of bench time, no discards.
- Teacher's final error: median 14 µm, p90 28 µm, 194/200 within 50 µm.
- At the near pause the detected tip was usually 5–7 px from the model (2–3 px in earlier batches).
  This may be drift since the 2026-11-11 power cycle. Recheck the calibration before the next bench
  evaluation.

Offline test on it (`outputs/finish_policy_offline/20261111-153242/`, crop, ring moved 266 µm):

| At the near hold (ceiling 0.98) | Old data, crop | New data, crop | New data, crop_aux |
|---|---|---|---|
| Marker gain | 0.26 | **0.56** [0.52, 0.60] | **0.61** [0.58, 0.64] |
| Chunk error at the holds (zero baseline) | 78 µm (108) | 46 µm (62) | 44 µm (62) |

- **The new data more than doubles the ring-following.** In it the whole move is set by the ring.
- The auxiliary remaining-path head (`crop_aux`) adds a little.
- At pre-touch the gain is 0.50–0.53 [IQR 0.0–0.7], against ~0.06 on the old data.
- **Still about 60% of the ceiling.**
  - Closed loop, a policy that closes 60% of the ring offset per chunk converges if it keeps
    re-reading the ring, about 0.4ⁿ after n chunks.
  - It will not converge if it touches down first.
  - So the bench is the real test.
**First bench trials (2026-11-11 evening, in-view start, `outputs/eval_auto/finish_2026-11_crop256_*`).**
- **Models.** Two ACT finish policies were trained on the finish batch (crop 256 px, tip actions,
  chunk 30, 40k steps; no auxiliary head):
  - `finish_2026-11_crop256_js`: crop + joint state;
  - `finish_2026-11_crop256_img`: crop only.
- **How each trial ran** (`eval_auto` `FINISH_START`):
  - it starts like the finish demos: tip in view, 0.1–1.5 mm beside the ring, 0.5–1.5 mm up
    (seeds 5,600,000 up);
  - the chunk is re-planned every 15 steps.
- **Not scored with `score_eval`.** The errors below come from `results.jsonl`:
  - vision is the detected tip against the target pixel;
  - FK is the kinematic tip projected through the calibration of 8 Nov.

| Run | Trials | Vision: median, ≤ 50 µm | FK: median, ≤ 50 µm | How the trials ended |
|---|---|---|---|---|
| crop + joints, policy height (`_js_act_finish_n15_*-184455` ep0, `-184654` ep1–9) | 10 | 30 µm, 9/10 | 45 µm, 6/10 | 7 stopped by the press limit (tip 82–99 µm into the dish), 2 settled 31 and 58 µm up, 1 time limit |
| crop only, scripted descent (`_img_act_finish_zscripted_n15_*-185701`) | 1 | 45 µm | 69 µm | settled at the surface |

- **Across the image, the finish policy works.** 9/10 trials ended within 50 µm by the detector,
  against A's 420 µm.
  - This start is easier than A's: these trials start in view and inside the training range, while
    A started from the scripted pre-touch point.
  - The pre-touch comparison is still to run (bench session in `NEXT.md`, runs c and d).
- **Height is the weak axis, even with the joint state.** The demos end exactly at the dish:
  - The demos' summed `tip_z` matches each start height to within about 1 µm (median 0.4 µm).
  - They slow down from about 30 µm per step to zero over the last 8 steps, then hold still.
  - The policy instead reaches the surface still descending at 6–26 µm per step and keeps going,
    until the 100 µm press limit stops it.
  - Its stopping height is wrong by about ±100 µm either way: 7 too low, 2 too high.
- **Likely reason.** 10 µm of height is about 0.003° of joint motion, a tiny fraction of the
  joints' normalised range. The policy cannot place the stop to that precision from the joints, and
  the top-down image barely shows height (theory 6).
  - **Consequence:** height should come from the kinematics and the dish height map (`FINISH_Z =
    "scripted"`), and the policy should control only the move across the image.
  - `eval_auto` now also locks the move across the image within `FINISH_LATERAL_LOCK_MM` (20 µm) of
    the touch, so a late correction cannot drag the tip along the dish.
- **Calibration drift.** The detected tip sits on average (+3.5, +0.2) px (about 45 µm) from the FK
  tip, consistent with the 5–7 px seen while recording the batch. The vision column is therefore
  the more trustworthy one. A full recalibration comes first in the next session.

**Bench session (2026-11-12, after a full recalibration).**
- **Recalibration.** Pivot 0.045 mm rms; scope fit 2.11 px rms over 13 points; verify median
  2.0 px, max 2.4 px; height map 12 µm rms, tilt 0.29°. The dish now reads 28.395 mm, 68 µm lower
  than the 8 Nov map. The tip model (`bench.yaml`, `tool.stl`) was refitted. Since then FK and
  vision agree to a few px.
- **Setup.** All four runs used scripted descent (`FINISH_Z = "scripted"`), the 20 µm lateral lock
  and a re-plan every 15 steps, with 10 trials each.
- **Errors** are from `results.jsonl`; `score_eval` and by-eye checks have not been run.

| Run | Start | Vision: median, ≤ 50 µm | FK: median, ≤ 50 µm | Folder (`outputs/eval_auto/`) |
|---|---|---|---|---|
| (a) crop only | in view (finish seeds) | 51 µm, 5/10 | 54 µm, 5/10 | `finish_2026-11_crop256_img_act_finish_zscripted_n15_20261112-161347` |
| (b) crop + joints | in view (finish seeds) | 38 µm, 7/10 | 41 µm, 6/10 | `..._js_act_finish_zscripted_n15_20261112-162348` |
| (c) crop only | scripted pre-touch | 2.0 mm, 1/10 | 1.9 mm, 1/10 | `..._img_act_handover_zscripted_n15_20261112-163349` (ep0–3) and `-170439` (ep4–9) |
| (d) crop + joints | scripted pre-touch | 2.1 mm, 0/10 | 2.0 mm, 0/10 | `..._js_act_handover_zscripted_n15_20261112-171139` |
| For reference: A | scripted pre-touch, 5 trials | | 420 µm | 6 Nov |
| For reference: (b)'s model, own height (old calibration) | in view | 30 µm, 9/10 | 45 µm, 6/10 | 11 Nov, 7/10 pressed into the dish |

**What the in-view runs (a, b) show.**
- From an in-range start, the finish policy lands at a median of 40–50 µm, with every trial
  touching at the surface.
  - That is 8–10× better than A's handover, but still 3× the teacher's 14 µm.
- **The misses are the policy settling in the wrong place, not the harness.**
  - In 9 of the 10 trials of (a), and all of (b) but one, the lateral motion had already stopped
    (≤ 0.5 µm/step) before the lock engaged.
  - The exception is (b) trial 4. The lock cut a correction still moving at 32 µm/step, and it
    ended at 159 µm.
  - Targets 3 and 9 were missed by both models (54–99 µm).
- **The joint state helped slightly here:** 7/10 against 5/10, a median of 38 against 51 µm. That
  is the opposite of theory 3's prediction, but 10 trials each cannot separate them.
- **Scripted height costs little or nothing.** (b)'s model with its own height scored 9/10 by
  vision on the same targets, but it pressed 80–100 µm into the dish in 7 of those, under the old
  calibration. That is not a clean comparison.

**What the handover runs (c, d) show.**
- **The pre-touch point is outside the finish policy's range in 8 of 10 trials.**
  - The tip starts 1.7–4.2 mm from the target, against at most 1.5 mm in the demos. The crop
    reaches only 1.7 mm either side of the target.
  - It also starts 1.2–2.8 mm up, against at most 1.5 mm.
  - The cause is geometric: the pre-touch point sits 1–4 mm back along the tilted pipette, plus
    the decoy offset, and the scripted descent is vertical.
- **From those 8 starts neither model makes progress.** The crop-only model wanders 80–800 µm and
  ends where it started. The joint-state model barely moves (24–150 µm).
- **The two in-range starts worked:**
  - trial 4, 1.46 mm off: crop only 28 µm; crop + joints 1.2 mm;
  - trial 8, 0.71 mm off: 82 µm and 89 µm.
- **Comparing with A's 420 µm is therefore not meaningful.** A drove along the pipette with its
  joint state. The finish policy only works once something has put the tip within about 1.5 mm of
  the ring and inside the crop.

**Incident.** Run (c) hung in trial 4, at a CUDA sync point inside `ACT.forward`, for 19 minutes.
The arm held still above the dish. The process was killed, CUDA tested fine, and the run resumed
from trial 4 with `FIRST_EPISODE = 4`. This has not been seen before.

**Fixed-point probe (12 Nov, `scripts/meca500/probe_finish_fixed_point.py`, offline).**
- **Question.** In runs (a) and (b) the misses had stopped moving sideways (≤ 0.5 µm/step) while
  still 4–9 px off, though the policy re-read the image every 15 steps. Two explanations:
  - *bias*: the policy follows a moved ring, but its own "on target" is a few px from the real one;
  - *deadband*: it does not respond to offsets below several px at all.
- **Method.** On the last frame each bench trial's policy saw, and on 50 near-hold frames from the
  finish batch, draw the ring at the target and moved by 2, 4, 8 and 16 px, crop as live, and
  measure how the predicted chunk changes. Also measure how much of the needed correction (the
  detector's tip-to-target vector) the unshifted chunk makes.
- **Result** (`outputs/train/finish_2026-11_crop256_img/checkpoints/last/probe_finish_fixed_point.csv`):

  | | Correction made toward the true target | Slope, ring moved 2–4 px | Slope, 16 px |
  |---|---|---|---|
  | Bench settled frames, both models | 3% (median) | 0.2–0.3 | about 0.5 |
  | Finish-batch near frames | 54–86% | 0.3 | 0.5–0.56 |

- **What it means.** Not a deadband: the policy responds to a ring moved by a few px, weakly. But at
  the frame where it settled its chunk is about zero while the tip is 1–8 px off, in a different
  direction each trial. So its implicit tip-to-ring estimate is wrong by a few px, frame by frame,
  and the low slope means it never notices. The same frames give the explicit regressor of theory 2
  (crop_tuned) about 1 px.

**Refinement test (12 Nov, bench).**
- **Hypothesis.** The residual error is the policy's imprecise implicit tip estimate. An explicit
  learned estimate, used for the last lateral correction, should bring run (a)'s 51 µm median to
  about 15–25 µm on the same targets.
- **Localiser.** `scripts/meca500/train_tip_localiser.py`: ResNet18 on a 192 px crop with the ring
  drawn, regressing the detector's tip minus the ring centre, trained on 1818 detection frames from
  600 episodes (all four datasets, finish batch included) with ring-shift augmentation.
  - Held out by episode: median 13 µm, p90 30 µm; at near 7–14 µm.
  - On the final frames of runs (a) and (b), it matched the detector to a median of 1.0 px (p90
    2.0 px), including the 5–9 px misses the policy ignored.
  - Model: `outputs/tip_localiser/20261112-184056/`.
- **Harness** (`eval_auto` `REFINE_MODEL`, code in `scripts/meca500/tip_localiser.py`). The scripted
  descent stops 50 µm above the touch. With the arm still, the localiser's estimate (median of
  8 frames) is corrected by a straight move at 5 µm/step, up to 4 times, until it is below 1 px.
  Then the tip descends with the lateral motion locked. The policy's own lateral actions are dropped
  from the hover on.
- **Result** (`outputs/eval_auto/finish_2026-11_crop256_img_act_finish_zscripted_refine_n15_20261112-184940`,
  crop-only model, same 10 targets as run a):

  | Target (seed 5,600,000 +) | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
  |---|---|---|---|---|---|---|---|---|---|---|
  | Run (a), vision µm | 23 | 15 | 43 | 74 | 60 | 104 | 42 | 115 | 19 | 96 |
  | With refinement, vision µm | 39 | 14 | 30 | 26 | 22 | 36 | 22 | 22 | 10 | 22 |
  | Localiser's first estimate at the hover, px | 1.0 | 0.7 | 3.5 | 5.6 | 4.1 | 6.7 | 0.4 | 7.1 | 0.8 | 6.2 |

  - **10/10 within 50 µm, median 22 µm, worst 39 µm**, against 5/10 and 51 µm. All five misses of
    run (a) were fixed. FK error is 7/10 within 50 µm, median 39 µm, which is calibration scatter
    (see "Is 50 µm a fair bar").
  - The prediction held. Theory 1, in its narrowed form, is the best explanation of the finish
    residual.
- **Touchdown shift.** Between the last estimate at the hover (all under 1 px) and the final frame,
  the tip moved (+0.7, −1.0) px on average (sd about 1 px), mostly forward along the pipette. That is
  the tip sliding on contact, about 16 µm, and it is most of the remaining 22 µm.
- **Caveat.** The localiser learned the detector's idea of "the tip", and the vision score uses the
  same detector. The by-eye review is the independent check.

**Next.**
- **Get the by-eye verdicts** from the review page and score every run against them.
- **Correct the touchdown slide:** aim the hover correction about 1 px behind the target along the
  pipette, or measure once more after contact. Prediction: median under 15 µm.
- **Make the handover land inside the finish range** (theory 4):
  - descend along the pipette axis to about 1 mm up, so the lateral offset is only the decoy's;
  - or let the approach policy with the tip watcher stop once the tip is within about 1 mm.
- **Or widen the finish policy's range:** a finish batch with starts up to 3–4 mm off and 3 mm up,
  with a larger crop or the full frame for the far part.
- **Fold the localiser into the policy**, so the system is one network: the ACT finish policy with
  the localiser's offset as an input, or the `crop_aux` auxiliary head trained to the same 1 px.
- **Use more of the camera:** a 192 px crop taken from the binned 2748×1836 image and downscaled 2×
  (about 6.7 µm/px) instead of from the 427×640 frame.

### 2. At 50 µm scale the full frame is too coarse (weakened by the 2026-11-10 test)

**Status (12 Nov): weakened, with headroom left.** An explicit regressor on a 192 px crop of our
427×640 frames localises the tip to about 1 px (13 µm), which was enough for 10/10 within 50 µm.
The frames are 8.6× downsampled from the sensor and the optics resolve about 2× finer than our
pixels, so a higher-resolution crop is available if 1 px stops being enough.

**Theory.**
- 50 µm is about 4 px.
- ACT's ResNet18 turns the 427×640 frame into a 14×20 grid of tokens, one per 32 px, so each token
  covers about 425 µm.
- Telling where the tip is relative to the ring is a sub-token question.
- Nothing in the architecture keeps fine position. There is no spatial softmax, no crop and no
  high-resolution branch.

**For.**
- All seven probed models have pre-touch gain ≤ 0.12, with every input set, action space and loss
  weight we tried.
- The response to the marker is about the same as, or smaller than, the response to unrelated
  texture.
- E's ring-localisation loss did not raise the pre-touch gain.
- From an on-target start, A finishes at 420 µm, about one token.

**Against / unknown.**
- C and D, the image-only models, read the marker a little (0.11–0.12).
- E centred the tip to 48–88 µm across from the handover. That says some fine signal gets through
  when the joints are absent.
- We have not checked whether the ring loss actually converged on the ring's position.

**Solution.**
- Make the finish a separate policy. It takes over once the tip is in view and is given:
  - a full-resolution crop, for example 128×128 px, centred on the target;
  - no joint state;
  - tip-motion actions in the camera's axes.
- Centring the crop on the target makes the task "move the tip to the crop centre", which is the same
  everywhere in the field.
- Add a keypoint or spatial-softmax head, or a pretrained patch encoder such as DINOv2, to keep fine
  position.

**Experiment (offline, about 30 min of GPU, no arm).** The script is
`scripts/meca500/probe_tip_localisation.py`.
- **Data:** 1218 raw frames from 400 episodes of the dataset, decoy and recovery sets, with the ring
  drawn as the policy sees it.
  - Each frame's label is our detected tip minus the target pixel.
  - Median offsets: pre-touch 1480 µm, mid 749 µm, near 278 µm, final 15 µm.
- **Method:** each variant regresses that vector with the same small head, using 5 folds split by
  episode. The variants:
  - (a) `policy_frozen`: the decoyrec policy's own ResNet18, frozen;
  - (b) `full_tuned`: ImageNet ResNet18 fine-tuned on the full frame (ACT's image pathway, when
    finding the tip is its only job);
  - (c) `crop_tuned`: the same network on a 192 px full-resolution crop centred on the target.
- **How to read it:** compare each variant with the "zero" baseline (predict the tip on the target)
  at the near checkpoint, which is where the 50 µm decision is made.
  - (a) bad, (b) good: the architecture can see it, and the policy never learnt to look. That points
    to theory 1, not this one.
  - (b) bad, (c) good: resolution and pooling are the bottleneck, and the crop policy is the fix.
  - All bad: the labels or the detector are the problem. Check `samples.csv`.

**Result (2026-11-10, `outputs/probe_tip_localisation/20261110-152102/`).** Held-out error of the
predicted tip-to-target vector, in µm:

| | Near: median | Near: within 50 µm | Pre-touch: median | All: median |
|---|---|---|---|---|
| Zero baseline (tip on the target) | 278 | 11% | 1480 | 273 |
| (a) the policy's frozen encoder | 69 | 32% | 315 | 70 |
| (b) ResNet18 tuned on the full frame | 33 | 76% | 179 | 35 |
| (c) ResNet18 tuned on the 192 px crop | 17 | 94% | 251 | 23 |

What it means:
- **The strong form of this theory is ruled out.** The policy's own encoder already holds the
  tip-to-target vector to about 70 µm at near: a 4× improvement on the baseline, using a head trained
  on 1000 frames.
  - Yet the policy's actions follow the ring with a gain of only 0.015 there.
  - So the information is in the encoder and the decoder doesn't use it. That is theory 1.
- **Resolution still matters for the last step.**
  - The same architecture tuned for the job gets 33 µm at near.
  - The crop gets 17 µm, with 94% of frames within 50 µm.
  - So a crop is the better input for a 50 µm finish, but the full frame is not what stops the
    policy today.
- **Caveats.**
  - Labels are our detector's tip, so about 1 px (13 µm) is the floor, and the networks may be
    learning the detector's quirks.
  - The frames are raw PNGs, while the policy trained on AV1 video.
  - The RMS columns (in `summary.txt`) are dominated by a few frames whose labels are far off,
    including some final frames. The medians are robust to them.

### 3. The joint state is still a shortcut, so the image is never needed

**Status (12 Nov): not supported for the finish.** On the bench, the crop + joint-state finish policy
scored 7/10 within 50 µm against 5/10 for crop only (10 trials each, not separable). Out of range,
from the pre-touch point, the joint-state model barely moved (24–150 µm) where the crop-only one
wandered, consistent with joints anchoring it to the demos' states. The offline (i)/(ii) comparison
below was replaced by this bench test.

**Theory.**
- With decoys up to 1.6 mm from the target, the pre-touch joint state still predicts the target to
  835 µm RMS.
- A policy that reads the joints and goes to the average decoy lands roughly there. That matches the
  no-decoy model's constant ~2 mm and A's "average path".
- This is the causal-confusion / copycat failure (de Haan et al. 2019; Wen et al. 2020).

**For.**
- The leak numbers above.
- The no-decoy model's error had the size and direction its leak predicts.

**Against.**
- Dropout 0.3 did not help.
- The image-only models C, D and E, which have no joints at all, also ignore the ring at pre-touch
  (gain ≤ 0.12).
- So the shortcut is not the whole story. It is a contributor, which is why this is third.

**Solution.**
- No joint state in the finish policy. Theory 2's design already does this.
- If the approach policy keeps the joints, retrain it on data collected at the current base position:
  the base moved 1.5 mm on 2026-11-08.

**Experiment.**
- With theory 2's crop finish policy, train two versions on the same data and compare their probe
  gains and held-out tip-to-target error:
  - (i) crop only;
  - (ii) crop + joint state.
- Prediction: (ii) is worse, with lower gain. This needs no bench time.

### 4. The approach is open-loop and inherently random, so it arrives 2–3 mm off

**Status (12 Nov): open, and now the main obstacle.** With the finish in range solved, the system
fails only where the tip starts outside the finish policy's 1.5 mm range. Even the scripted
pre-touch point is outside it in 8/10 trials (1.7–4.2 mm off), so the approach has to aim for
"tip within about 1 mm of the ring", not for the pre-touch point.

**Theory.**
- The teacher picks 1–2 random via-points and a random speed in each episode. None of that is
  visible in the first frame or the joints.
- So the coarse path cannot be predicted from the observation. ACT, with its latent set to zero at
  inference, learns the average path.
- Each 30-step chunk then runs 1.5 s blind while the tip is out of view.

**For.**
- On training episodes, teacher-forced, the predicted chunks differ from the recorded ones by about
  2 mm per chunk.
- Precision entry is 2.1–2.7 mm off, which is worse than aiming at the average target (1.5 mm).
- Re-planning every 10 steps changed this only from 2.70 to 2.37 mm.
- 12 of 20 decoyrec approaches ended with the tip out of view.

**Against.**
- None that we know of. But this only explains the approach. It matters for "on its own"; it does
  not explain the finish.

**Solution.**
- **Make the approach target deterministic:** with the start and target given, the path is a
  function of what the policy sees. This means no random via-points, or via-points the policy can
  observe, such as a phase variable.
- **Or keep the randomness and stop needing the approach to be accurate:** the approach only has to
  bring the tip into view, about 1 mm, and the finish policy does the rest.
- For actions, use absolute joint targets or tip targets relative to the target, so errors don't
  accumulate the way summed deltas do.

**Experiment.**
- Record 100 episodes with via-points fixed by the target, so the path is deterministic. This is a
  `record_auto` config change, about 1.5 h unattended.
- Train the same ACT and compare with A on two measures:
  - the teacher-forced chunk error (offline);
  - "tip in view at the end of the approach" on 10 bench trials (today 57%).
- Prediction: the chunk error drops from ~2 mm to a few hundred µm, and the tip is in view in ≥ 90%.

### 5. The policy leaves the demos' states and cannot recover

**Theory.**
- Once the approach is 2–3 mm off, the policy is in states no demo visited. The expert's precision
  entry was at most 0.67 mm off.
- Behaviour cloning then compounds the error (Ross et al. 2011).
- Model A's handover runs also stopped in a state no demo had: with the ring under the tip but
  0.45–1.5 mm above the dish.

**For.**
- The hover endings.
- The off-distribution precision-entry numbers above.

**Against.**
- Adding 100 off-course and hover recovery episodes (decoyrec) did not give a visibly better approach:
  12/20 still needed the search.
- With 100 examples spread over a large state space, that test is weak.

**Solution.**
- Use DAgger-style data. Roll the policy out on the bench, then have the scripted teacher (or ASR)
  record the correction from wherever it ended.
- The bench resets itself, so a few hundred of these are about a day's unattended collection.

**Experiment.**
- Run 2 rounds of 50 policy rollouts, with each ending relabelled by a scripted correction recording,
  then retrain.
- Compare precision-entry distance and tip-in-view rate over 10 bench trials, against decoyrec.
- Do this only after theories 1, 2 and 4 are addressed. Otherwise the new data teaches the same
  ignoring of the ring.

### 6. Image-only models cannot tell the tip's height

**Status (12 Nov): supported, and worked around.** With height from the kinematics and the dish map
(`FINISH_Z = "scripted"`), all 30 in-range finish trials of 12 Nov touched within 2 µm of the
mapped surface. Their errors are lateral only.

**Theory.**
- A top-down microscope image barely changes with the tip's height near focus.
- Without the joint state, the policy cannot tell 0.15 mm from 1.5 mm above the dish.

**For.**
- E centred the tip but hovered 1.1–1.5 mm up.
- In its own approach, E descended 12–24 mm short.
- **Since 2026-11-11:** the crop + joint-state finish policy also misjudged its stopping height by
  about ±100 µm. It ran 82–99 µm into the dish in 7/10 and stopped 31–58 µm up in 2. So at this
  scale the joint state does not fix it either (theory 1's bench trials).

**Against.**
- This doesn't apply to the coarse joint-state approach, which only needs millimetres.

**Solution.**
- Split the axes. The finish policy controls the tip's position across the image, and the height
  comes from kinematics: lower to the measured dish height, as ASR's lowering already does.
- Alternatively, feed only the tip height (one number) as state, not all six joints.

**Experiment.**
- Run E (or the crop finish policy) with in-image actions only, and a scripted vertical lower
  and touch, in 10 handover trials.
- Prediction: 48–88 µm across becomes a real touch at about that distance.

---

## Limits that matter only below about 100 µm

These do not explain a 2 mm miss. But a policy that fixes theories 1–5 will run into them.

- **Label noise.** The teacher aims through the camera calibration (1.18 px, i.e. 16 µm, rms
  residual). The dataset's own final error has a median of 15.9 µm, and 155/156 are within 50 µm in
  E2–E4. A perfect imitator inherits that.
- **What "the tip" is.** ASR's tip point sits about 20–23 px (about 0.3 mm) back from our free end.
  Training labels, the servo and the scoring must all use one definition.
- **Base re-seat.** Since 2026-11-08, the base sits 1.5 mm from where all training data was recorded.
  Any joint-state policy run now sees that offset, so retrain or re-seat first.
- **Scoring drift.** `score_eval.py` projects through the current calibration. Re-scoring old runs
  today changes their view and without-search numbers.
- **Pixel scale (12 Nov).** 13.4 µm/px in our frames, so 50 µm is 3.7 px; FK cannot resolve it
  (calibration residual 28 µm rms) and the image can (detector about 1 px). See "Is 50 µm a fair
  bar" at the top.
- **Touchdown slide (12 Nov).** The tip moves about 1.2 px (16 µm), mostly forward along the
  pipette, during the last 50 µm and the contact. It is now the largest error left in the refined
  finish.

## Suggested order

| Step | Covers | Arm needed | Rough time |
|---|---|---|---|
| Offline perception test (done 2026-11-10) | Theory 2 | No | 30 min |
| Finish policy on existing finish frames, probed (done 2026-11-10/11) | Theories 1 and 3 | No | 1 day + training |
| Finish-only data batch (done 2026-11-11, 200 eps) | Theory 1 | Yes, unattended | 3 h |
| Finish-policy bench trials, in-view start (done 2026-11-12: median 38–51 µm) | Theories 1, 3 and 6 | Yes | 1.5 h |
| Handover bench trials of the finish policy, from pre-touch (done 2026-11-12: pre-touch is out of range) | Theories 1, 2 and 6 | Yes | included above |
| Fixed-point probe and learned refinement (done 2026-11-12: 10/10, median 22 µm) | Theories 1 and 2 | Yes | 2 h |
| By-eye verdicts on the review page, re-score every run (next) | Scoring | No | 20 min |
| Touchdown-slide correction (next) | Finish residual | Yes | 30 min |
| Handover that lands within 1.5 mm, or a wider finish batch (next) | Theory 4 | Yes | 1–3 h |
| Deterministic-path approach data and retrain | Theory 4 | Yes, unattended | 1.5 h + training |
| DAgger rounds | Theory 5 | Yes, unattended | 1 day |

Retrain or re-seat for the 1.5 mm base offset before any joint-state policy goes on the bench.
