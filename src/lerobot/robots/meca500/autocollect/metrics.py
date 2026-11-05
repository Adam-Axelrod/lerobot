"""Motion metrics for the results table, the same for a policy, a controller and a person."""

from __future__ import annotations

import numpy as np


def resample(t: np.ndarray, x: np.ndarray, fs: float) -> np.ndarray:
    """``x`` (n, d) sampled at uneven times ``t``, linearly interpolated at ``fs`` Hz."""
    grid = np.arange(t[0], t[-1], 1 / fs)
    return np.column_stack([np.interp(grid, t, x[:, i]) for i in range(x.shape[1])])


def sparc(speed: np.ndarray, fs: float, padlevel: int = 4, fc: float = 10.0, amp_th: float = 0.05) -> float:
    """Spectral arc length of a speed profile (Balasubramanian et al., 2015). Closer to 0 is smoother."""
    speed = np.asarray(speed, float)
    if len(speed) < 4 or not np.any(speed):
        return float("nan")
    nfft = int(2 ** (np.ceil(np.log2(len(speed))) + padlevel))
    freq = np.arange(0, fs, fs / nfft)
    mag = np.abs(np.fft.fft(speed, nfft))
    mag = mag / mag.max()
    keep = freq <= fc
    freq, mag = freq[keep], mag[keep]
    above = np.nonzero(mag >= amp_th)[0]
    freq, mag = freq[above[0] : above[-1] + 1], mag[above[0] : above[-1] + 1]
    if len(freq) < 2:
        return float("nan")
    return float(-np.sum(np.sqrt((np.diff(freq) / (freq[-1] - freq[0])) ** 2 + np.diff(mag) ** 2)))


def path_metrics(t: np.ndarray, tip_mm: np.ndarray, fs: float = 20.0, arrive_um: float = 50.0) -> dict:
    """Arrival time, economy of movement and smoothness of a tip path (mm, base frame).

    ``arrival_s`` is when the tip reached its final position and stayed within ``arrive_um``. It is
    measured against the tip's own end point rather than the target, because the forward kinematics
    are 60-120 um off in the dish plane: good for the shape of a path, not for a 50 um target
    decision. ``economy`` is straight-line distance over distance travelled.
    """
    t, tip_mm = np.asarray(t, float), np.asarray(tip_mm, float)
    if len(t) < 4:
        return {"arrival_s": float("nan"), "economy": float("nan"), "sparc": float("nan")}
    to_end = np.linalg.norm(tip_mm - tip_mm[-1], axis=1) * 1000
    away = np.nonzero(to_end > arrive_um)[0]
    arrival = float(t[away[-1] + 1] - t[0]) if len(away) and away[-1] + 1 < len(t) else 0.0
    even = resample(t, tip_mm, fs)
    travelled = float(np.sum(np.linalg.norm(np.diff(even, axis=0), axis=1)))
    direct = float(np.linalg.norm(even[-1] - even[0]))
    speed = np.linalg.norm(np.diff(even, axis=0), axis=1) * fs
    return {
        "arrival_s": arrival,
        "economy": direct / travelled if travelled > 0 else float("nan"),
        "sparc": sparc(speed, fs),
    }
