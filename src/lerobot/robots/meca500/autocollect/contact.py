"""Detect the pipette tip touching the dish with the Bota F/T sensor.

Measured on Huanyu's setup (MiniONE Pro, bota_probe.py 2026-10-14):

- force noise: 0.03-0.08 N raw, 0.005-0.009 N with ~10 Hz smoothing, plus ~0.02 N/min drift.
  A light hand-guided touch is buried in it;
- torque noise: ~0.05-0.1 mNm. Touches show as clean 0.3-1 mNm steps, because the tip sits
  ~125 mm from the sensor: 0.01 N at the tip -> ~1.25 mNm at the sensor. Torque is ~10x more
  sensitive to a touch than force.

So contact = the smoothed TORQUE moving away from a baseline taken just before a descent. The
descent is a pure translation, so the tool's gravity torque stays constant and any change is
contact. Hard force/torque limits stand behind it as a backstop.

The sensor returns repeated frames (~485 distinct frames/s of ~1.4k reads), so samples are
de-duplicated before filtering.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np


class WrenchStream:
    """Reads a bota_driver.BotaDriver on a background thread into a rolling buffer.

    ``sensor`` must already be configured and activated. Each read is followed by a sub-ms
    sleep: back-to-back reads at the sensor's rate starve other threads of the GIL (the Basler
    feed froze in the first bota_probe run).
    """

    def __init__(self, sensor, buffer_s: float = 30.0, rate_hz: float = 800.0):
        self.sensor = sensor
        self._buf: deque = deque(maxlen=int(buffer_s * rate_hz))
        self._lock = threading.Lock()
        self._running = True
        self._last = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            frame = self.sensor.read_frame()
            time.sleep(0.0003)  # release the GIL after every read
            if not frame:
                continue
            w = (*frame.force[:3], *frame.torque[:3])
            # The driver hands back the same frame repeatedly; skip those by the sensor's own
            # timestamp. (Comparing VALUES would also drop genuinely new frames whenever the
            # reading is momentarily unchanged, starving the baseline of samples.)
            key = getattr(frame, "timestamp", None)
            key = w if key is None else key
            if key == self._last:
                continue
            self._last = key
            with self._lock:
                self._buf.append((time.perf_counter(), *w))

    def window(self, seconds: float) -> np.ndarray:
        """(n, 7) array of (t, fx, fy, fz, tx, ty, tz) from the last ``seconds``."""
        with self._lock:
            a = np.array(self._buf) if self._buf else np.empty((0, 7))
        return a[a[:, 0] >= time.perf_counter() - seconds] if len(a) else a

    def mean(self, seconds: float) -> np.ndarray | None:
        """Mean wrench (fx, fy, fz, tx, ty, tz) over the last ``seconds``, or None if empty."""
        a = self.window(seconds)
        return a[:, 1:].mean(axis=0) if len(a) else None

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)


@dataclass
class ContactThresholds:
    torque_mnm: float = 0.3  # contact: smoothed |dT| above this (~2-3 mN at a 125 mm tip)
    smooth_s: float = 0.1  # averaging window (~10 Hz)
    force_limit_n: float = 0.05  # backstop: smoothed |dF| above this always stops
    torque_limit_mnm: float = 3.0  # backstop: smoothed |dT| above this always stops


@dataclass
class ContactReading:
    contact: bool
    d_torque_mnm: float
    d_force_n: float
    reason: str = ""


class ContactDetector:
    """Compares the smoothed wrench with a baseline captured at the start of a move."""

    def __init__(self, stream: WrenchStream, thresholds: ContactThresholds | None = None):
        self.stream = stream
        self.th = thresholds or ContactThresholds()
        self.baseline: np.ndarray | None = None

    def capture_baseline(self, seconds: float = 0.5) -> np.ndarray:
        """Average the wrench while the arm is STILL, just before descending."""
        b = self.stream.mean(seconds)
        if b is None:
            raise RuntimeError("No force/torque samples - is the sensor streaming?")
        self.baseline = b
        return b

    def check(self) -> ContactReading:
        if self.baseline is None:
            raise RuntimeError("capture_baseline() first")
        m = self.stream.mean(self.th.smooth_s)
        if m is None:
            # no fresh data is itself unsafe while moving toward the dish
            return ContactReading(True, float("nan"), float("nan"), "no sensor data")
        return evaluate(m, self.baseline, self.th)


def evaluate(smoothed: np.ndarray, baseline: np.ndarray, th: ContactThresholds) -> ContactReading:
    """Pure decision rule, separated out for testing."""
    d = smoothed - baseline
    df = float(np.linalg.norm(d[:3]))
    dt = float(np.linalg.norm(d[3:]) * 1000.0)
    if df > th.force_limit_n:
        return ContactReading(True, dt, df, "force limit")
    if dt > th.torque_limit_mnm:
        return ContactReading(True, dt, df, "torque limit")
    if dt > th.torque_mnm:
        return ContactReading(True, dt, df, "torque")
    return ContactReading(False, dt, df)
