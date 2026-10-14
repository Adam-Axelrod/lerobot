"""Measure the Bota force/torque sensor's noise and the size of a pipette touch, jogging with the
SpaceMouse in the same script.

For the auto-calibration's contact detection: how noisy is the sensor at rest, and how big is the
force when the pipette tip touches the dish? One script does it all - SpaceMouse jogging (as in
teleop; Enter toggles fine mode), the microscope feed, and the live force in the window title:

    1. after start-up the sensor is tared; keep your hands off for the first BASELINE_S seconds
       (title shows BASELINE) -> the noise at rest
    2. jog the tip down in fine mode until it lightly touches the dish, hold ~2 s, lift off
    3. repeat the touch 2-3 times, then press q (or close the window)

Keys (window focused):  t  re-tare and restart the baseline    q  finish

It prints a summary - per-axis noise (std) at rest, and the peak |change| during the touches,
which is where a contact threshold must sit between - and saves the raw samples.

The sensor config comes from the rig (rigs.py BOTA_SENSORS): the original setup has a MiniONE,
Huanyu's setup a MiniONE Pro, each with its own JSON in src/lerobot/robots/meca500/bota/. Owns the
arm and the camera: close teleop / other scripts and the web portal first.
"""

import logging
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
from live_view import LiveView
from rigs import BOTA_SENSORS, axis_signs, bota_config_path, require_home_joints, rig_config

from lerobot.robots.utils import make_robot_from_config
from lerobot.teleoperators.meca500_spacemouse.config_meca500_spacemouse import Meca500SpacemouseConfig
from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins

REPO = Path(__file__).resolve().parents[2]
# ----------------------------- CONFIG -----------------------------
RIG = "meca500_basler"
CONFIG = bota_config_path(RIG)
BASELINE_S = 5.0
GAIN_TR, GAIN_ROT = 20.0, 15.0  # coarse jog, mm/s and deg/s
GAIN_TR_FINE, GAIN_ROT_FINE = 1.0, 1.0  # fine jog (Enter) - use it near the dish
# -----------------------------------------------------------------

WINDOW = "bota_probe  (touch the dish lightly 3x; t = re-tare, q = finish)"


class SensorReader:
    """Reads the Bota sensor on a background thread so jogging and display stay smooth."""

    def __init__(self, sensor):
        self.sensor = sensor
        self.rows: list[tuple] = []  # (t, fx, fy, fz, tx, ty, tz), t from the last (re)tare
        self.latest = np.zeros(6)
        self.t0 = time.perf_counter()
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            frame = self.sensor.read_frame()
            if not frame:
                time.sleep(0.0005)  # no new frame yet; do not spin a core
                continue
            w = np.array([*frame.force[:3], *frame.torque[:3]], dtype=float)
            with self._lock:
                self.latest = w
                self.rows.append((time.perf_counter() - self.t0, *w))

    def retare(self) -> None:
        with self._lock:
            if not self.sensor.tare():
                logging.warning("Re-tare failed; keeping the previous zero.")
            self.rows = []
            self.t0 = time.perf_counter()

    def elapsed(self) -> float:
        return time.perf_counter() - self.t0

    def snapshot(self) -> np.ndarray:
        with self._lock:
            return np.array(self.rows)

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


def summarise(a: np.ndarray) -> None:
    if len(a) == 0:
        sys.exit("\nNo frames received.")
    rate = len(a) / max(a[-1, 0], 1e-9)
    base = a[a[:, 0] < BASELINE_S, 1:]
    rest = a[a[:, 0] >= BASELINE_S, 1:]
    print(f"\n{len(a)} frames over {a[-1, 0]:.1f} s ({rate:.0f} Hz)")
    if len(base):
        print(
            f"baseline noise std  F {np.round(base[:, :3].std(axis=0), 5).tolist()} N   "
            f"|F| {np.linalg.norm(base[:, :3] - base[:, :3].mean(0), axis=1).std():.5f} N"
        )
        print(f"baseline drift      F mean {np.round(base[:, :3].mean(axis=0), 5).tolist()} N")
    if len(rest) and len(base):
        df = np.linalg.norm(rest[:, :3] - base[:, :3].mean(0), axis=1)
        print(
            f"after baseline      peak |dF| {df.max():.4f} N, 99th pct {np.percentile(df, 99):.4f} N, "
            f"peak per axis {np.round(np.abs(rest[:, :3] - base[:, :3].mean(0)).max(axis=0), 4).tolist()} N"
        )
    out = REPO / "outputs" / "bota_probe.npy"
    out.parent.mkdir(exist_ok=True)
    np.save(out, a)
    print(f"raw samples saved to {out} (t, fx, fy, fz, tx, ty, tz) - send this file to Claude if asked")


def main() -> None:
    import bota_driver

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(message)s")
    register_third_party_plugins()
    print(f"Rig {RIG}: {BOTA_SENSORS[RIG]} sensor, config {CONFIG}")

    robot = make_robot_from_config(replace(rig_config(RIG), monitor_mode=True))
    teleop = make_teleoperator_from_config(
        Meca500SpacemouseConfig(
            id="meca500_spacemouse",
            gain_tr=GAIN_TR,
            gain_rot=GAIN_ROT,
            gain_tr_fine=GAIN_TR_FINE,
            gain_rot_fine=GAIN_ROT_FINE,
            axis_signs=axis_signs(RIG),
            home_joints=require_home_joints(RIG),
        )
    )
    # Window first: if the display cannot open, nothing has connected to the arm yet.
    view = LiveView(WINDOW)
    sensor = bota_driver.BotaDriver(str(CONFIG))
    if not sensor.configure():
        view.close()
        sys.exit("Failed to configure the Bota driver - check the COM port / product in the config file.")
    robot.connect()
    teleop.connect()
    reader = None
    try:
        # Tare once the arm is connected and still, so the zero is the resting tool weight.
        if not sensor.tare():
            sys.exit("Failed to tare the sensor.")
        if not sensor.activate():
            sys.exit("Failed to activate the sensor.")
        reader = SensorReader(sensor)
        print(f"Sensor active and tared. Hands off for {BASELINE_S:.0f} s, then touch the dish 3 times.\n")
        while not view.closed:
            teleop.get_action()
            obs = robot.get_observation()
            w = reader.latest
            t = reader.elapsed()
            phase = (
                f"BASELINE {BASELINE_S - t:.0f}s - hands off" if t < BASELINE_S else "touch the dish lightly"
            )
            view.show(
                obs["microscope_cam"],
                title=f"{WINDOW}\n{phase}   F [{w[0]:+.3f} {w[1]:+.3f} {w[2]:+.3f}] N   |F| {np.linalg.norm(w[:3]):.3f} N",
                crosshair=False,
            )
            key = view.key()
            if key == "t":
                reader.retare()
                print("Re-tared; baseline restarted - hands off.")
            elif key == "q":
                break
    finally:
        view.close()
        if reader is not None:
            reader.stop()
        try:
            sensor.deactivate()
            sensor.shutdown()
        except Exception:
            pass
        teleop.disconnect()
        robot.disconnect()

    summarise(reader.snapshot() if reader is not None else np.empty((0, 7)))


if __name__ == "__main__":
    main()
