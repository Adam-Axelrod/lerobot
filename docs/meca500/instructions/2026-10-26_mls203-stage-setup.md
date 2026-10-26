# Connect the MLS203 stage to this PC

**Bench:** Huanyu's setup (rig `meca500_basler`)
**Time needed:** about 20 min
**Why:** `record_auto.py` will move the dish to a random position between episodes, so the policy
never sees the same patch of dish twice. The scope calibration maps camera pixels to robot positions,
and moving the dish under a fixed camera does not change that mapping.

## 1. Install Thorlabs Kinesis
1. Download **Kinesis, 64-bit** from thorlabs.com (Software → Motion Control → Kinesis). This
   PC's Python is 64-bit; the 32-bit build will not load.
2. Install it to the default folder. The Python code expects
   `C:\Program Files\Thorlabs\Kinesis\`.

## 2. Connect and test in the Kinesis app
> **Raise the pipette first.** Send the arm home (or check that it is already there). Homing
> drives the stage to its limits, carrying the dish with it, and it must not drag under the tip.

1. Connect the stage controller to this PC by USB and power it on. Huanyu's setup probably uses
   a **BBD302**, but check the label on the front.
2. Open **Kinesis**. The controller should appear, with two channels (X and Y).
3. Write down:
   - the controller **model** (BBD302 / BBD202 / BBD102 / other);
   - its **serial number**, shown in Kinesis (a BBD302 serial starts with `103`);
   - the **position readout** (mm) of each channel after homing.
4. **Home both channels** in Kinesis.
5. Jog each axis by about 1 mm while watching the microscope feed. Note which way the image moves.
6. **Close Kinesis.** Only one program can hold the controller at a time, so the app has to be
   closed before the Python scripts can connect.

## 3. Find a good centre position
With Kinesis open again (or by noting the readouts):
1. Move the stage until a clear part of the dish is under the objective. That will be the
   centre the random positions are spread around.
2. Write down that **X, Y position in mm**.
3. Move about ±5 mm from it on each axis and check that the dish is still visible everywhere.
   Avoid the dish rim and any thick labels.

## Send Claude
- the controller model and serial number;
- the centre X, Y in mm, and how far (in mm) you can move from it and still be well inside the dish;
- which direction the image moves for +X and for +Y;
- whether homing worked without errors.

Claude will then add `pythonnet`, a small stage driver, and a test script (`stage_probe.py`) that
moves the stage while showing the microscope feed. Its first test runs against the Kinesis
simulator, with no hardware involved.
