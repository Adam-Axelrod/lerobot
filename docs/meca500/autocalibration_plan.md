# Auto-calibration (Huanyu's setup first)

**Goal:** replace the two manual calibrations (`calibrate_tip.py`, `calibrate_scope.py`) with an
automatic sequence. They have to be redone after **every pipette change or knock**, and
manually that takes about 35 min of careful jogging and clicking.

## Building blocks

| Block | How | Status |
|---|---|---|
| **Find the tip in the image** | Background subtraction: a frame with the pipette backed out of view, minus a frame with it in view, gives the pipette as the only change. The tip is the end of that blob furthest from where it enters the illuminated circle. `src/lerobot/robots/meca500/autocollect/tip_detect.py` | **Done.** Correct on the last frame of 12/12 of Huanyu's Basler episodes, despite clutter and a sticker beside the tip. 4 synthetic tests. Not yet seen the *new* pipette. |
| **Detect the touch** | Bota F/T sensor between flange and pipette: tare, then stop a slow guarded descent when \|ΔF\| exceeds a threshold set from the measured noise. | Probe script `scripts/meca500/bota_probe.py` written. **Needs a probe run** (sensor model, port, noise, contact size). |
| **Guarded descent** | Control mode. Creep down at about 0.2 mm/s (velocity command, or short `MoveJoints` steps from IK along -z) while reading the sensor at about 500 Hz. Stop on contact, then record joints. Abort if travel exceeds about 2 mm below the expected dish or force exceeds a hard limit. | To do |
| **Background frame** | Back the pipette 20 mm out along its own axis (larger than the 5.5 × 8.3 mm field), so the tip and shank leave the view. | To do |

## Sequences

**Auto scope calibration:** for each of about 12–16 grid targets across the field (positions
from the current calibration):
1. background frame;
2. move to hover 1 mm above (collision-checked);
3. guarded descent;
4. frame, then detect the tip;
5. record the pixel and the tip XY/Z from FK;
6. lift.

Then fit, report, save (the same file format as `calibrate_scope.py`).

**Auto tip calibration:**
1. At about 6–8 collision-checked wrist orientations spanning at least 40° around the touch
   orientation, visually servo the tip onto one image point (detect → pixel error → XY
   correction through the calibration → move, repeat until under 2 px).
2. Guarded touch, then record the flange pose.
3. Run the pivot fit, then save `tool.measured_tip_in_flange`.

**Combined:** tip → re-import → scope → (optionally) tip again, until they agree. Finish with an
automatic version of `check_tip.py`.

## Safety
- every move goes through the planner's collision checker;
- slow speeds within 5 mm of the dish;
- a hard force limit triggers a stop and retract;
- a maximum descent depth;
- start from and end at home, with the arm holding its pose on disconnect.

## Open questions for the user
1. ~~Which Bota sensor?~~ Answered: MiniONE Pro on Huanyu's setup (MiniONE on the original). Both configs are
   in `src/lerobot/robots/meca500/bota/`, chosen per rig by `rigs.py`.
2. The maximum acceptable contact force for the glass pipette (suggested stop at 0.05–0.1 N).
