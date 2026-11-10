# Why the learned approach does not reach 50 µm

**Last updated:** 2026-11-10. Evidence is from bench runs on 5–8 Nov at the 0 mm base position,
plus the offline perception test of 10 Nov, which swapped theories 1 and 2.

## What the policy does

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
- If it stays low on the existing data, record one batch of 200 short finish episodes and retrain.
  That is about 1 h of bench time.
- Bench test: 10 handover trials from the scripted pre-touch point, comparing with A's 420 µm.

### 2. At 50 µm scale the full frame is too coarse (weakened by the 2026-11-10 test)

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

**Theory.**
- A top-down microscope image barely changes with the tip's height near focus.
- Without the joint state, the policy cannot tell 0.15 mm from 1.5 mm above the dish.

**For.**
- E centred the tip but hovered 1.1–1.5 mm up.
- In its own approach, E descended 12–24 mm short.

**Against.**
- This doesn't apply to the joint-state models.

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

## Suggested order

| Step | Covers | Arm needed | Rough time |
|---|---|---|---|
| Offline perception test (done 2026-11-10) | Theory 2 | No | 30 min |
| Finish policy on existing finish frames, probed | Theories 1 and 3 | No | 1 day + training |
| Finish-only data batch, if needed | Theory 1 | Yes, unattended | 1–2 h |
| Handover bench trials of the finish policy | Theories 1, 2 and 6 | Yes | 1 h |
| Deterministic-path approach data and retrain | Theory 4 | Yes, unattended | 1.5 h + training |
| DAgger rounds | Theory 5 | Yes, unattended | 1 day |

Retrain or re-seat for the 1.5 mm base offset before any joint-state policy goes on the bench.
