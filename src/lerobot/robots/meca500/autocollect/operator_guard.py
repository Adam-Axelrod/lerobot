"""Keeps a person on the SpaceMouse off the dish and out of the collision model.

The teleop commands velocity straight to the controller, with none of the checks a planned path
or a policy command gets. The guard runs in its velocity loop:

- **Dish floor.** Downward speed is limited in proportion to the tip's height above the dish
  height map and is zero at the surface, so an operator lowers the tip until it stops. The speed
  limit is what makes the floor safe: the joint readings are a few ms old, and at full coarse
  speed that alone would be most of a millimetre.
- **Collision model.** The check a policy command gets. Past ``trip_below_mm`` the guard trips:
  velocity is zero until ``reset`` and the trial ends as a collision. The check takes several ms,
  so it runs at ``clearance_period_s``, not on every cycle. A person cannot rotate, and on 1 of 40
  test seeds the fixed-orientation route via the planner's gate point came 0.73 mm inside the
  5 mm tool margin, so the trip point allows 1 mm: about 4 mm of real clearance remains.
"""

from __future__ import annotations

import time

import numpy as np


class OperatorGuard:
    def __init__(
        self,
        planner,
        descent_gain_per_s: float = 5.0,
        min_descent_mm_s: float = 0.1,
        clearance_period_s: float = 0.05,
        trip_below_mm: float = -1.0,
    ):
        self.planner = planner  # read live: the stage move changes its dish and surface height
        self.descent_gain_per_s = descent_gain_per_s
        self.min_descent_mm_s = min_descent_mm_s
        self.clearance_period_s = clearance_period_s
        self.trip_below_mm = trip_below_mm
        self.check_clearance = True
        self.tripped: str | None = None
        self.floor_hit = False
        self._next_check = 0.0

    def reset(self) -> None:
        self.tripped = None
        self.floor_hit = False
        self._next_check = 0.0

    def height_mm(self, q) -> float:
        return float(self.planner.kin.tip(np.asarray(q, float))[2, 3] - self.planner.calib.surface_z_mm)

    def filter(self, q, v_mm_s: np.ndarray) -> np.ndarray:
        """The allowed tip velocity (base frame, mm/s) at joints ``q`` for the requested one."""
        if self.tripped:
            return np.zeros(3)
        now = time.perf_counter()
        if self.check_clearance and now >= self._next_check:
            self._next_check = now + self.clearance_period_s
            clear = self.planner.checker.clearance(np.asarray(q, float))
            if clear.min_mm < self.trip_below_mm:
                self.tripped = (
                    f"stopped: {clear.where} within margin of {clear.obstacle} ({clear.min_mm:.1f} mm)"
                )
                return np.zeros(3)
        v = np.array(v_mm_s, dtype=float)
        if v[2] < 0:
            height = self.height_mm(q)
            if height <= 0:
                self.floor_hit = True
                v[2] = 0.0
            else:
                v[2] = max(v[2], -max(height * self.descent_gain_per_s, self.min_descent_mm_s))
        return v
