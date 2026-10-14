# Bota force-sensor probe (for auto-calibration)

**Bench:** Huanyu's setup
**Time needed:** about 10 min
**Why:** the auto-calibration detects the pipette touching the dish with the Bota F/T sensor.
To set its contact threshold, Claude needs the sensor's noise at rest and the size of a real,
light touch. Background: [`../autocalibration_plan.md`](../autocalibration_plan.md).

## Questions first (just answer in the chat)
1. **Which sensor is fitted now?** The repo's `bota_sensor_config.json` says
   **BFT-MIPS-ENET-CG on COM5**, but the CAD shows a **MiniONE Pro**. What's the model, and is
   it plugged in by **USB** (which COM port, from Device Manager → Ports) or by **Ethernet**?
2. **Contact force limit:** what's the most force you'd accept on the glass pipette? The
   suggestion is to stop at **0.05–0.1 N**.

## The probe run
The probe only reads the sensor; it never talks to the arm, so it runs alongside teleop.

1. Terminal 1, teleop as usual:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\teleoperate_microscope.py
   ```
2. Jog the tip to just above the dish (in view, about 1 mm up), and **let go of the
   SpaceMouse**.
3. Terminal 2, the probe:
   ```powershell
   .venv\Scripts\python.exe scripts\meca500\bota_probe.py
   ```
   It tares the sensor, then prints live forces.
4. **Don't touch anything for the first 5 s** (the screen shows `BASELINE`); that measures the
   noise.
5. Then, in fine mode, lower the tip until it **just touches** the dish, hold about 2 s, and lift
   off. Do this **3 times**, as lightly as you can.
6. Press **Ctrl-C** in the probe terminal.
7. **Send Claude** the summary lines (frame rate, baseline noise, peak |dF|). The raw samples are
   saved to `outputs\bota_probe.npy`; Claude may ask for that file.

### If something goes wrong
| Symptom | What to do |
|---|---|
| "Failed to configure the Bota driver" | Wrong COM port or product in `bota_sensor_config.json`. Tell Claude the model and port, and it'll make a matching config. |
| No frames, or 0 Hz | The sensor isn't streaming. Check the cable and power, and close any other program using it. |
