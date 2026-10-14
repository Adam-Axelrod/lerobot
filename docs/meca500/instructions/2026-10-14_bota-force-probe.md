# Bota force-sensor probe (for auto-calibration)

**Bench:** Huanyu's setup
**Time needed:** about 10 min
**Why:** the auto-calibration detects the pipette touching the dish with the Bota F/T sensor.
To set its contact threshold, Claude needs the sensor's noise at rest and the size of a real,
light touch. Background: [`../autocalibration_plan.md`](../autocalibration_plan.md).

## Questions first (just answer in the chat)
1. ~~Which sensor is fitted?~~ **Answered:** Huanyu's setup has a **MiniONE Pro** (BFT-MIPS-SER-CG, COM8). The repo now keeps both configs (`src/lerobot/robots/meca500/bota/`) and picks the right one per rig, so the probe uses the MiniONE Pro automatically.
2. **Contact force limit:** what's the most force you'd accept on the glass pipette? The
   suggestion is to stop at **0.05–0.1 N**.

## The probe run
One script does everything: SpaceMouse jogging (as in teleop; **Enter** toggles fine mode), the
microscope feed, and the live force in the window title. **Close teleop and the web portal
first**, since the probe drives the arm itself.

1. Run it:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\bota_probe.py
   ```
   It connects to the arm and camera, then tares the sensor.
2. **Hands off for the first 5 s.** The title shows `BASELINE ... hands off`; this measures the
   noise. If you bump something, click the window and press **t** to re-tare and restart the
   baseline.
3. When the title says *touch the dish lightly*, jog the tip down in fine mode until it **just
   touches** the dish, hold about 2 s, and lift off. Watch `|F|` in the title rise. Do this
   **3 times**, as lightly as you can.
4. Press **q** (or close the window).
5. **Send Claude** the summary lines (frame rate, baseline noise, peak |dF|). The raw samples are
   saved to `outputs\bota_probe.npy`; Claude may ask for that file.

### If something goes wrong
| Symptom | What to do |
|---|---|
| "Failed to configure the Bota driver" | The COM port changed (a different USB socket). Check Device Manager → Ports and fix `com_port` in `src/lerobot/robots/meca500/bota/miniONE_pro.json`. |
| No frames, or 0 Hz | The sensor isn't streaming. Check the cable and power, and close any other program using it. |
