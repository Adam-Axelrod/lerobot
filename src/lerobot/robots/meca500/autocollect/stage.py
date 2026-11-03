"""The MLS203 XY dish stage (Thorlabs BBD302 controller) and where it puts the dish.

Moving the stage between episodes moves the dish under the fixed camera, so the policy never
sees the same patch of dish twice. Two parts:

- ``MLS203Stage``: the driver, through the Kinesis .NET libraries (pythonnet). It never homes on
  its own: homing sweeps the dish to the middle of the travel, so the caller does it with the arm
  parked. It moves gently, because the dish rests loose on the acrylic plate.
- ``StageModel``: where the dish is, in the robot frame, for a stage position. It comes from the
  measured image shift per stage mm and the scope calibration's pixel -> robot map, so the
  collision model can place the dish wall for each episode.

Kinesis must be closed: only one program can hold the controller.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

KINESIS_DIR = r"C:\Program Files\Thorlabs\Kinesis"
# Gentle moves: the default 100 mm/s at 1000 mm/s^2 (0.1 g) risks sliding the loose dish.
MAX_VELOCITY_MM_S = 10.0
ACCELERATION_MM_S2 = 50.0
MOVE_TIMEOUT_MS = 60_000
HOME_TIMEOUT_MS = 90_000


class StageError(RuntimeError):
    pass


class MLS203Stage:
    """Two-channel MLS203 on a BBD302 (channel 1 = X, channel 2 = Y), positions in mm."""

    def __init__(self, serial: str, velocity_mm_s: float = MAX_VELOCITY_MM_S, accel_mm_s2: float = ACCELERATION_MM_S2):
        self.serial = str(serial)
        self.velocity, self.accel = velocity_mm_s, accel_mm_s2
        self._dev = None
        self._ch: list = []

    def connect(self) -> None:
        import clr  # pythonnet

        for dll in ("DeviceManagerCLI", "GenericMotorCLI", "Benchtop.BrushlessMotorCLI"):
            clr.AddReference(rf"{KINESIS_DIR}\Thorlabs.MotionControl.{dll}.dll")
        from System import Decimal  # type: ignore
        from Thorlabs.MotionControl.Benchtop.BrushlessMotorCLI import BenchtopBrushlessMotor  # type: ignore
        from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI  # type: ignore

        self._decimal = Decimal
        DeviceManagerCLI.BuildDeviceList()
        if self.serial not in list(DeviceManagerCLI.GetDeviceList()):
            raise StageError(f"Stage controller {self.serial} not found. Is it on and plugged in, and is Kinesis closed?")
        self._dev = BenchtopBrushlessMotor.CreateBenchtopBrushlessMotor(self.serial)
        self._dev.Connect(self.serial)
        for i in (1, 2):
            c = self._dev.GetChannel(i)
            if not c.IsSettingsInitialized():
                c.WaitForSettingsInitialized(5000)
            c.StartPolling(100)
            time.sleep(0.3)
            c.LoadMotorConfiguration(c.DeviceID)
            if not c.IsEnabled:
                # After a power cycle both channels start disabled. On 2026-11-03 the touchscreen
                # would not enable X, but this call did.
                c.EnableDevice()
                time.sleep(1.0)
            if not c.IsEnabled:
                raise StageError(f"Stage channel {i} will not enable. Power-cycle the BBD302 and retry.")
            v = c.GetVelocityParams()
            v.MaxVelocity, v.Acceleration = Decimal(self.velocity), Decimal(self.accel)
            c.SetVelocityParams(v)
            self._ch.append(c)

    @property
    def is_homed(self) -> bool:
        return all(c.Status.IsHomed for c in self._ch)

    def home(self) -> None:
        """Home both axes. The dish moves to the middle of the travel: park the arm first."""
        for c in self._ch:
            c.Home(HOME_TIMEOUT_MS)
        if not self.is_homed:
            raise StageError("Stage did not report homed after homing.")

    def position(self) -> np.ndarray:
        return np.array([float(str(c.Position)) for c in self._ch])

    def move_to(self, xy) -> np.ndarray:
        """Blocking move of both axes; returns the position reached."""
        if not self.is_homed:
            raise StageError("Stage is not homed; positions mean nothing. Home it with the arm parked.")
        for c, v in zip(self._ch, xy, strict=True):
            c.MoveTo(self._decimal(float(v)), MOVE_TIMEOUT_MS)
        reached = self.position()
        if np.abs(reached - np.asarray(xy, float)).max() > 0.01:
            raise StageError(f"Stage reached {reached.tolist()}, not {list(xy)}.")
        return reached

    def disconnect(self) -> None:
        for c in self._ch:
            try:
                c.StopPolling()
            except Exception:  # noqa: BLE001 - best effort on the way out
                pass
        if self._dev is not None:
            self._dev.Disconnect(False)
        self._dev, self._ch = None, []


@dataclass
class StageModel:
    """Where the dish is in the robot frame for a stage position, and which positions to use.

    ``reference_xy``: the stage position the bench's dish obstacles are written for.
    ``robot_per_stage_mm``: 2x2, robot (x, y) mm moved per stage (X, Y) mm.
    ``centre_xy`` and ``radius_mm``: random positions are drawn in this disc (bench.yaml
    stage_random_centre_mm, stage_random_radius_mm: chosen to keep the approach clear of the wall).
    ``travel``: the stage's limits, mm.
    """

    reference_xy: np.ndarray
    robot_per_stage_mm: np.ndarray
    centre_xy: np.ndarray
    radius_mm: float
    travel: tuple[tuple[float, float], tuple[float, float]] = ((0.0, 110.0), (0.0, 75.0))

    @classmethod
    def from_bench(cls, scope: dict, calib) -> StageModel:
        """From bench.yaml's ``scope`` section and the scope calibration.

        The image shift per stage mm (px) times the calibration's pixel -> robot map gives how far
        the dish moves in the robot frame per stage mm.
        """
        px_per_mm = np.asarray(scope["stage_px_per_mm"], float)
        mm_per_px = np.asarray(calib.px_to_xy, float)[:, :2]
        return cls(
            reference_xy=np.asarray(scope["stage_xy_mm"], float),
            robot_per_stage_mm=mm_per_px @ px_per_mm,
            centre_xy=np.asarray(scope.get("stage_random_centre_mm", scope["stage_xy_mm"]), float),
            radius_mm=float(scope.get("stage_random_radius_mm", 0.0)),
        )

    def dish_offset(self, stage_xy) -> np.ndarray:
        """Robot-frame (x, y, 0) shift of the dish from where bench.yaml puts it."""
        d = self.robot_per_stage_mm @ (np.asarray(stage_xy, float) - self.reference_xy)
        return np.array([d[0], d[1], 0.0])

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        """A uniform random stage position in the disc, inside the travel."""
        for _ in range(100):
            r, a = self.radius_mm * np.sqrt(rng.uniform()), rng.uniform(0, 2 * np.pi)
            xy = self.centre_xy + r * np.array([np.cos(a), np.sin(a)])
            if all(lo <= v <= hi for v, (lo, hi) in zip(xy, self.travel, strict=True)):
                return xy
        raise StageError("No stage position inside the travel; check stage_random_centre_mm and the radius.")
