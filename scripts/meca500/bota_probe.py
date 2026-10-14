"""Stream the Bota force/torque sensor and summarise its noise and a contact. READ-ONLY.

For the auto-calibration's contact detection: how noisy is the sensor at rest, and how big is
the force when the pipette tip touches the dish? Run it ALONGSIDE teleoperate_microscope.py
(the sensor is a separate device; this never talks to the arm):

    1. leave the arm still for the first ~5 s  -> noise baseline (after the tare)
    2. jog the tip down in fine mode until it lightly touches the dish, hold ~2 s, lift off
    3. repeat the touch 2-3 times, then Ctrl-C

It prints a live line, then a summary: per-axis noise (std) at rest, and the peak |change|
seen, which is what a contact threshold must sit between.

The sensor config comes from the rig (rigs.py BOTA_SENSORS): the original setup has a MiniONE,
Huanyu's setup a MiniONE Pro, each with its own JSON in src/lerobot/robots/meca500/bota/.
"""

import sys
import time
from pathlib import Path

import numpy as np
from rigs import BOTA_SENSORS, bota_config_path

REPO = Path(__file__).resolve().parents[2]
# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
CONFIG = bota_config_path(RIG)
BASELINE_S = 5.0
PRINT_HZ = 10.0
# -----------------------------------------------------------------


def main() -> None:
    import bota_driver

    print(f"Rig {RIG}: {BOTA_SENSORS[RIG]} sensor, config {CONFIG}")
    sensor = bota_driver.BotaDriver(str(CONFIG))
    if not sensor.configure():
        sys.exit("Failed to configure the Bota driver - check the COM port / product in the config file.")
    if not sensor.tare():
        sys.exit("Failed to tare the sensor.")
    if not sensor.activate():
        sys.exit("Failed to activate the sensor.")
    print(f"Sensor active and tared. Keep the arm still for {BASELINE_S:.0f} s, then touch the dish.\n")

    t0 = time.perf_counter()
    last_print = 0.0
    rows = []  # (t, fx, fy, fz, tx, ty, tz)
    try:
        while True:
            frame = sensor.read_frame()
            if not frame:
                time.sleep(0.0005)  # no new frame yet; do not spin a core
                continue
            t = time.perf_counter() - t0
            w = [*frame.force[:3], *frame.torque[:3]]
            rows.append((t, *w))
            if t - last_print >= 1.0 / PRINT_HZ:
                last_print = t
                phase = "BASELINE" if t < BASELINE_S else "        "
                print(
                    f"\r{phase} t {t:6.1f}s  F [{w[0]:+.4f} {w[1]:+.4f} {w[2]:+.4f}] N  |F| {np.linalg.norm(w[:3]):.4f}  "
                    f"T [{w[3]:+.5f} {w[4]:+.5f} {w[5]:+.5f}] Nm   ",
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        try:
            sensor.deactivate()
            sensor.shutdown()
        except Exception:
            pass

    a = np.array(rows)
    if len(a) == 0:
        sys.exit("\nNo frames received.")
    rate = len(a) / max(a[-1, 0], 1e-9)
    base = a[a[:, 0] < BASELINE_S, 1:]
    rest = a[a[:, 0] >= BASELINE_S, 1:]
    print(f"\n\n{len(a)} frames over {a[-1, 0]:.1f} s ({rate:.0f} Hz)")
    if len(base):
        print(
            f"baseline noise std  F {np.round(base[:, :3].std(axis=0), 5).tolist()} N   "
            f"|F| {np.linalg.norm(base[:, :3] - base[:, :3].mean(0), axis=1).std():.5f} N"
        )
        print(f"baseline drift      F mean {np.round(base[:, :3].mean(axis=0), 5).tolist()} N")
    if len(rest) and len(base):
        df_norm = np.linalg.norm(rest[:, :3] - base[:, :3].mean(0), axis=1)
        print(
            f"after baseline      peak |df_norm| {df_norm.max():.4f} N, 99th pct {np.percentile(df_norm, 99):.4f} N, "
            f"peak per axis {np.round(np.abs(rest[:, :3] - base[:, :3].mean(0)).max(axis=0), 4).tolist()} N"
        )
    out = REPO / "outputs" / "bota_probe.npy"
    out.parent.mkdir(exist_ok=True)
    np.save(out, a)
    print(f"raw samples saved to {out} (t, fx, fy, fz, tx, ty, tz) - send this file to Claude if asked")


if __name__ == "__main__":
    main()
