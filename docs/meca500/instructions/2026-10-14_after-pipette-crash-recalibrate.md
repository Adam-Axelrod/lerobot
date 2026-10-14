# After the pipette crash: re-measure the tip, re-calibrate the scope, rerun hover mode

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Time needed:** about 40 min at the arm, plus a few minutes of Claude's work in between
**Branch:** `feat/meca500-autocollect`

## What happened, and what's fixed
At the end of the first 3-episode hover run, the scripts **deactivated** the arm when they
disconnected. Deactivation drops the brakes, so the arm sagged, and the pipette, hovering 5 mm
over the dish, was driven into it and bent.

Both causes are fixed in the code (pushed):
- **Disconnecting no longer deactivates the arm.** It stops, holds its pose and stays powered,
  and the next script picks it up without re-homing. This applies to every script: record,
  inference and reset.
- **`record_auto.py` drives home at the end.** After the last episode, or when you press Esc, it
  backs the pipette out and goes home before disconnecting.

**Why recalibrate:** a bent or replaced pipette puts the tip somewhere new. The collision model,
the scope calibration and the touch pose all depend on where the tip is, so they all need
redoing. From now on, do Parts A and B **after every pipette change or knock**.

---

## Before you start
1. Ethernet cable on Huanyu's bench, **web portal closed**, nothing else controlling the arm.
2. SpaceMouse and Basler plugged in, a spare dish on the stage, a hand near the e-stop.
3. `cd C:\Users\aa24\PhD\lerobot`

## Step 0: The pipette
- If it's visibly bent, cracked or chipped, **replace it**.
- If it looks straight, you can keep it. Part A will show whether the tip has moved.

---

## Part A: Re-measure the tip (`calibrate_tip.py`, about 15 min)
```powershell
.venv\Scripts\python.exe scripts\meca500\calibrate_tip.py
```
1. Pick a small, sharp feature on the dish.
2. In fine mode (Enter), touch it **as lightly as possible**, at the first moment of contact,
   centred on the yellow crosshair and in focus. Click the window to give it focus, then press
   **c**.
3. Lift, then **tilt or twist the wrist by 20–30°, in a different direction each time**. Last
   session the spread stopped growing at 33° after capture 3. Bring the tip back to the same
   feature and press **c**.
4. Take **6–8 captures**, aiming for an orientation spread of **40° or more**.
5. Press **f**. A good fit has a **residual of 0.3 mm or less**. If one capture looks off, use
   **u** to undo it and retake it.
6. Press **s**. This now saves the **measured tip position** to `bench.yaml`
   (`tool.measured_tip_in_flange`).
7. **Send Claude the fit report.** Claude then re-imports the tool model with the measured tip,
   and re-checks the collision model, the touch and start poses, and the dry run.

## Part B: Re-calibrate the scope (`calibrate_scope.py`, about 20 min)
**Wait until Claude confirms the re-import**, because this calibration uses the tip position.
Then follow **Part A** of
[`2026-10-14_scope-calibration-and-first-hover-run.md`](2026-10-14_scope-calibration-and-first-hover-run.md):
dish height with `z` (3–4 light touches), about 12 clicked points over the whole image, then `f`
(≤ 5 px), check the cyan cross, then `s`.

**Send Claude:** the `fit:` line, and one `joints [...]` line from the `z` samples.

## Part C: Hover run again (`record_auto.py`)
**Wait until Claude has rerun the checks.** Then:
1. In `scripts/meca500/record_auto.py`: `NUM_EPISODES = 3`, `STAGE = "hover"`.
2. Close teleop and the portal. The script homes the arm itself.
3. Run it with your hand on the e-stop:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\record_auto.py
   ```
4. **Watch the end of the run.** After the last episode the arm should back out and return
   home, then the script disconnects, and the arm should **stay where it is** (no sag).
5. Send Claude the terminal output, and say whether the arm held its pose after disconnecting.

### Troubleshooting
| Symptom | What to do |
|---|---|
| Arm sags or goes limp at the end | Hit the e-stop and tell Claude. It shouldn't happen any more. |
| "Could not return home ..." at the end | The arm holds its pose. Jog it clear with teleop, press H, and tell Claude. |
| "still paused/in error after ResumeMotion" | Release the e-stop, check the portal, close it, rerun. |
| calibrate_tip residual stays > 0.5 mm | Lighter touches, bigger and more varied tilts, and wait until the arm is still before pressing `c`. |
