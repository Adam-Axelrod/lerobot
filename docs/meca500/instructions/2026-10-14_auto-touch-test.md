# Auto-touch test: does the arm stop on contact by itself?

**Bench:** Huanyu's setup (rig `meca500_basler`, MiniONE Pro force sensor)
**Time needed:** about 10 min
**Do this after:** the scope recalibration (`calibrate_scope.py`) for the current pipette. The
test uses the calibrated dish height to decide where to hover and how far it may go.
Background: [`../autocalibration_plan.md`](../autocalibration_plan.md).

## What it does
The first fully automatic contact. `scripts/meca500/auto_touch_test.py`:
1. drives the arm **home**, then to a **hover pose 3 mm above the dish** at the centre of the
   field of view (both moves collision-checked);
2. holds still for 0.5 s to take a force/torque baseline;
3. **creeps straight down at 0.15 mm/s** while watching the torque;
4. **stops the instant it detects contact** (torque change above 0.3 mNm, about 2–3 mN at the
   tip), records the tip height, and lifts back to the hover pose;
5. repeats **5 times**, then returns home.

Safety backstops:
- it always stops if the force change exceeds **0.05 N**;
- it aborts if the tip goes **2 mm below** the calibrated dish without detecting contact;
- it never moves anywhere the collision model hasn't checked.

## The display
The microscope feed, with a banner across the top:

| Banner | Meaning |
|---|---|
| **Green: NO CONTACT** | Nothing detected |
| **Red: CONTACT (torque / force limit / …)** | Contact detected, and the reason |
| **Yellow bar** under the banner | Live torque change. The white tick in the middle is the contact threshold; the bar reaching it means contact. |

The window title shows the phase (moving home, settling, descending, lifting …) and the live
dT / dF values.

## Steps
1. Close teleop and the web portal. Put a spare dish on the stage. Keep a hand near the
   **e-stop**.
2. Run:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\auto_touch_test.py
   ```
3. **Watch the first descent closely.** The tip should come into focus, touch, and the banner
   should turn **red the moment it touches**, with the arm stopping straight away. If the tip
   visibly presses or bends without the banner turning red, press **q** at once (or the
   e-stop).
4. To stop at any time, click the window and press **q** (or close it). The arm stops and holds
   its pose.
5. **Send Claude** the terminal output. The last line gives the contact height against the
   calibrated dish, and the spread across the 5 trials.

## What a good result looks like
- 5 out of 5 trials end in **CONTACT (torque)**;
- the **spread under about 20 µm**, which shows the touch repeats;
- the mean contact height within a few tenths of a millimetre of the calibrated dish.

## If something goes wrong
| Symptom | What to do |
|---|---|
| The banner is already red before descending | The torque drifted or the arm vibrated. Press **q** and send Claude the output; the threshold may need adjusting. |
| "over-travel ... no contact" | It went 2 mm below the calibrated dish without feeling contact. Either the calibration is off or the threshold is too high. Send the output. |
| "Bota sensor failed to configure" | Check the COM port in `src/lerobot/robots/meca500/bota/miniONE_pro.json`. |
| The arm stutters while descending | Tell Claude; the display may be slowing the loop too much. |
