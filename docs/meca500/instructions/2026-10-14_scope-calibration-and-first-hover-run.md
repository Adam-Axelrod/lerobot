# Scope calibration and the first hover-mode run (Huanyu's setup)

**Bench:** Huanyu's setup (rig `meca500_basler`, Basler microscope camera)
**Time needed:** about 20 min for Part A, then about 15 min for Part C once Part B is done
**Branch:** `feat/meca500-autocollect`

You have three parts to do:
- **Part A** calibrates the microscope. It maps image pixels to robot positions and measures the real dish height.
- **Part B** is Claude's turn. Claude reruns the planning checks with your measurements.
- **Part C** is the first automated run, with the tip stopping 5 mm *above* the dish.

Already done and verified:
- **Arm model:** `check_kinematics.py` passed, matching the controller to under 0.001 mm.
- **Tip position:** measured with `calibrate_tip.py`; the CAD with the 4 mm mounting plate matches it to 0.62 mm.
- **Planner:** the dry run plans 100 of 100 episodes without collisions.

---

## Before you start (every session)

1. Move the Ethernet cable to Huanyu's bench.
2. **Close the Meca500 web page** (`http://192.168.0.100`). Only one program can control the arm at a time, and the web page counts as one. If a script says *"Another user is already controlling the robot"*, this is why.
3. Close any other script that drives the arm: teleop, record, or a previous calibration run.
4. Plug in the SpaceMouse and the Basler, and put a spare dish on the stage.
5. Keep a hand near the **e-stop**.
6. Open a terminal in the repo folder:
   ```powershell
   cd C:\Users\aa24\PhD\lerobot
   ```

### The calibration window
The microscope view opens in a **matplotlib window**:
- **Focus:** click on the window once to give it keyboard focus before pressing keys.
- **Zooming:** the toolbar's **magnifier** zooms in, which is useful for clicking the tip precisely. While zoom or pan is switched on, clicks do **not** record points. Click the magnifier again to switch it off.
- **Fine mode:** **Enter** toggles slow jogging, the same as in teleop. Use it whenever the tip is near the dish.

---

## Part A: Calibrate the microscope (`calibrate_scope.py`)

**What it does:** it learns which image pixel corresponds to which robot position at the dish surface. That's what turns a target drawn on the image into a place the robot can drive the tip to. It also measures the dish height.

**Why care matters:** this calibration's error ends up in every recorded episode, as the gap between where the target is drawn and where the tip actually lands.

```powershell
.venv\Scripts\python.exe scripts\meca500\calibrate_scope.py
```

### A1. Measure the dish height (key `z`)
1. In fine mode, lower the tip until it **just touches** the dish: the tip snaps into focus, or the dish starts to shift. **Don't press down**; the glass bends and adds error.
2. Press **`z`**. The terminal prints the height, plus a line like:
   ```
   joints [-56.2, 60.1, 22.3, 131.2, 83.0, -126.9]  <- usable as approach.touch_nominal_joints
   ```
3. Repeat at **3–4 different places** on the dish.
4. **Copy one of those `joints [...]` lines to Claude,** ideally one where the pipette angle looks like a natural approach. It replaces the provisional touch pose.

### A2. Click calibration points
1. Touch the tip lightly to the dish so it's in focus.
2. **Left-click on the very end of the tip** in the image (zoom in first if needed, then switch zoom off). A **red dot** marks the point, and the terminal prints it.
3. Lift slightly, move to a new spot, touch, and click again.
4. Collect **about 12 points spread over the whole image**: near all four corners, the middle of each edge, and the centre. A spread-out set matters more than the number of points.
5. If you mis-click, press **`u`** to undo the last point.

### A3. Fit, check, save
1. Press **`f`**. The terminal prints something like:
   ```
   fit: 1.8 px rms = 5.2 um rms, 2.9 um/px, surface z 8.61 mm (+/- 40 um over 4 samples)
   ```
2. **Target: ≤ 5 px rms.** The recorder refuses a worse calibration. If it's higher, undo (`u`) any obviously bad clicks, add points where coverage is thin, and press `f` again.
3. **Live check:** after a fit, a **cyan cross** shows where the calibration thinks the tip is. Jog the tip around the image, touching the dish. **The cyan cross should stay on the tip everywhere.** If it drifts in one area, add points there and fit again.
4. Press **`s`** to save. It writes `assets/benches/meca500_basler/scope_calibration.json` and quits.
5. **Send Claude:** the final `fit:` line, and one of the `joints [...]` lines from A1.

(`q` quits without saving.)

### If something goes wrong
| Symptom | What to do |
|---|---|
| "Another user is already controlling the robot" | Close the web page or other arm scripts, then rerun. |
| Keys do nothing | Click on the image window first to give it focus. |
| Clicks don't add red dots | The toolbar's zoom or pan is still on; click its button again. |
| "need >= 6 points and >= 1 surface sample" | Add more clicked points, and at least one `z`. |
| The arm behaves strangely after a crash | Power-cycle it, or reset it from the web page, then **close the web page** before rerunning. |

---

## Part B: Claude reruns the checks (no action from you)

Using your calibration and touch pose, Claude will:
1. rerun the touch-pose search with the measured dish height;
2. rerun the 100-episode dry run and send you the path viewer;
3. tell you when Part C is ready.

---

## Part C: First automated episodes, hover mode (`record_auto.py`)

**What it does:** the arm drives itself.
1. It makes an unrecorded move to a random start pose.
2. A green target marker appears on the microscope image.
3. It records the approach to the marker.

In **hover mode** the tip stops **5 mm above** the dish and everything runs at **half speed**. It doesn't touch the dish, and nothing is uploaded.

0. **Start from home.** The first move is planned from wherever the arm is, and it does *not*
   back the pipette out of the dish first. Run `teleoperate_microscope.py`, jog the tip **up and
   out of the stage opening**, then press **H** to go home, then close teleop (Ctrl-C).
   (Pressing H with the tip still in the dish is a direct joint move that could drag the tip.)
1. Open `scripts/meca500/record_auto.py` and check the settings near the top:
   ```python
   NAME = "basler_auto_2026-10"
   NUM_EPISODES = 3          # change from 50 for this first run
   STAGE = "hover"
   PUSH_TO_HUB = False
   ```
2. Run it:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\record_auto.py
   ```
3. **Watch every move with your hand on the e-stop.** These are the first motions the planner has ever sent to the real arm. Stop immediately if anything heads toward the illuminator or the stage edge.
4. Keys during recording:
   | Key | Action |
   |---|---|
   | **→** (right arrow) | End the current episode early. It's **discarded**, because a cut-short episode never reached its target. |
   | **←** (left arrow) | Discard and re-record. |
   | **Esc** | Stop the session. |
5. **Send Claude:**
   - the terminal output;
   - whether each move looked safe and sensible;
   - where the tip ended relative to the green marker (it should hover right over it, slightly out of focus because it's 5 mm up).

The run creates a local dataset `AdamAxelrod/basler_auto_2026-10`. If you need to rerun from scratch, either delete that folder under `%USERPROFILE%\.cache\huggingface\lerobot\AdamAxelrod\`, or set `RESUME = True` to append to it.

### After Part C
If hover mode looks right, the next instructions will cover touch mode on a spare dish, then unattended batches.
