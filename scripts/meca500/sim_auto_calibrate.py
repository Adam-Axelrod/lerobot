"""Offline check of auto_calibrate.py: its stages run against a SIMULATED bench. No hardware.

A fake Meca500 (driven through the real kinematics), a fake Basler (synthetic frames: the pipette
drawn where the TRUE tip is, blurred by its height above the dish, sliding forward along the
dish once it touches) and a fake Bota sensor (torque building up slowly past contact, so the
slide - not the torque - has to catch the touch, as on the arm). The model's tip is deliberately wrong by
TRUE_DELTA in the flange frame - like a bent or replaced pipette - so the stages must:
find the dish and the (possibly out-of-view) tip, recover the true tip by pivot calibration,
fit the scope, and verify. Prints the recovered tip vs the truth and the verify errors.

Writes nothing: the tip "save" only updates the in-memory model, and scope_calibration.json is
never touched. Run after changing auto_calibrate.py, arm_session.py, tip_detect.py or contact.py:

    python scripts/meca500/sim_auto_calibrate.py                 (pipette change: 3.3 mm tip error)
    python scripts/meca500/sim_auto_calibrate.py --same-pipette  (0.2 mm error, default 2 mm start)

(~80 s each; set MPLBACKEND=Agg to hide the window.)

Expected (2026-10-14): tip recovered to ~0.02 mm, scope fit ~0.3 px, dish z within 0.01 mm,
verify < 1 px.
"""

import itertools
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import auto_calibrate as autocal  # noqa: E402
from arm_session import Display  # noqa: E402
from live_view import LiveView  # noqa: E402

from lerobot.robots.meca500.autocollect.bench import BenchConfig  # noqa: E402
from lerobot.robots.meca500.autocollect.contact import (  # noqa: E402
    ContactDetector,
    ContactThresholds,
    WrenchStream,
)
from lerobot.robots.meca500.autocollect.kinematics import Meca500Kinematics  # noqa: E402

SAME_PIPETTE = "--same-pipette" in sys.argv
if SAME_PIPETTE:  # recalibrating the same pipette: tip barely moved, start at the default height
    TRUE_DELTA = np.array([0.1, -0.1, 0.2])
else:  # a pipette change: the tip is mm off, so start high, as the CONFIG comment says to
    TRUE_DELTA = np.array([1.0, -0.8, 3.0])
    autocal.START_ABOVE_DISH_MM = 6.0
TRUE_DISH_Z = 8.0
SPEEDUP = 5.0  # sim time runs faster during velocity moves (higher = coarser touch-downs)

bench = BenchConfig.load("meca500_basler")
model_kin = bench.kinematics()
true_tool = model_kin.tool.copy()
true_tool[:3, 3] += TRUE_DELTA
true_kin = Meca500Kinematics(tool=true_tool)
flange_kin = Meca500Kinematics()  # tool = flange, for moving the flange
cam = bench.calibration()  # the true camera
A_inv_lin = np.linalg.inv(np.asarray(cam.px_to_xy)[:, :2])


class SimMeca:
    def __init__(self, q):
        self.q = np.asarray(q, float)
        self.vel = np.zeros(3)
        self.t_last = time.perf_counter()

    def _advance(self):
        now = time.perf_counter()
        dt = (now - self.t_last) * SPEEDUP
        self.t_last = now
        if np.any(self.vel):
            f = flange_kin.flange(self.q)
            f[:3, 3] += self.vel * dt
            q = flange_kin.ik(f, self.q)
            if q is not None:
                self.q = q

    def GetRtJointPos(self, **k):  # noqa: N802 - mimics mecademicpy
        self._advance()
        return list(self.q)

    GetRtTargetJointPos = GetRtJointPos

    def MoveJoints(self, *q):  # noqa: N802 - mimics mecademicpy
        self._advance()
        self.q = np.array(q, float)

    def MoveLinVelWrf(self, x, y, z, *_):  # noqa: N802 - mimics mecademicpy
        self._advance()
        self.vel = np.array([x, y, z], float)

    def MoveLinRelWrf(self, dx, dy, dz, *_):  # noqa: N802 - mimics mecademicpy
        f = flange_kin.flange(self.q)
        f[:3, 3] += [dx, dy, dz]
        self.q = flange_kin.ik(f, self.q)

    def WaitIdle(self, timeout=None):  # noqa: N802 - mimics mecademicpy
        # Like the real controller: a checkpoint round trip takes time, so a short-timeout
        # WaitIdle fails. (The first arm run hung in a loop of 50 ms WaitIdle calls.)
        if timeout is not None and timeout < 0.08:
            raise TimeoutError("checkpoint not reached")
        time.sleep(0.08)
        self._advance()

    def GetStatusRobot(self):  # noqa: N802 - mimics mecademicpy
        return SimpleNamespace(error_status=False, pause_motion_status=False, error_code=0, error_msg="")

    def __getattr__(self, n):  # ClearMotion, ResumeMotion, SetCartLinVel, ...
        return lambda *a, **k: None

    # truth
    def raw_tip(self):
        self._advance()
        return true_kin.tip(self.q)

    def true_tip(self):
        """Where the real tip IS: once it reaches the dish it cannot go lower, so further descent
        slides it FORWARD along the dish in the pipette's direction, like the shallow glass
        pipette on the arm (2026-10-14)."""
        t = self.raw_tip()
        pen = TRUE_DISH_Z - t[2, 3]
        if pen > 0:
            axis = t[:3, 2]
            horiz = axis[:2] / max(np.linalg.norm(axis[:2]), 1e-9)
            elev = np.arcsin(np.clip(-axis[2], 0.05, 1.0))  # angle below horizontal
            t = t.copy()
            t[:2, 3] += horiz * pen / np.tan(elev)
            t[2, 3] = TRUE_DISH_Z
        return t


rng = np.random.default_rng(0)
BG = np.zeros((640, 427, 3), np.uint8)
# Like Huanyu's setup on the arm (2026-10-14): the bright field runs off the image edges.
cv2.circle(BG, (221, 362), 273, (240, 200, 60), -1)
for _ in range(300):
    cv2.circle(
        BG,
        (int(rng.integers(20, 410)), int(rng.integers(120, 520))),
        int(rng.integers(1, 3)),
        (90, 70, 20),
        -1,
    )


def render(meca):
    t = meca.true_tip()
    tip, h = t[:3, 3], t[2, 3] - TRUE_DISH_Z
    img = BG.copy()
    if h < 15:
        px = A_inv_lin @ (tip[:2] - np.asarray(cam.px_to_xy)[:, 2])
        back = tip[:2] - 30 * t[:2, 2]  # 30 mm back along the pipette, projected
        px_back = A_inv_lin @ (back - np.asarray(cam.px_to_xy)[:, 2])
        layer = img.copy()
        cv2.line(layer, (int(px[0]), int(px[1])), (int(px_back[0]), int(px_back[1])), (30, 25, 10), 7)
        s = 3 * max(h, 0)  # blur grows smoothly with height, as through a real objective
        k = int(2 * round(2 * s) + 1)
        layer = cv2.GaussianBlur(layer, (k, k), s) if s > 0.3 else layer
        img = layer
    return img


class Robot:
    def __init__(self, meca):
        self.robot = meca

    def get_observation(self):
        return {"microscope_cam": render(self.robot)}


meca = SimMeca(bench.section("start")["nominal_joints"])
ts = itertools.count()


class Sensor:
    def read_frame(self):
        time.sleep(0.0012)
        # Torque builds up only gradually with the descent past contact (the tip slides rather
        # than pushes): it reaches the 0.3 mNm threshold 0.2 mm past contact, so vision (the
        # slide) must catch the touch first, as on the arm.
        pen = TRUE_DISH_Z - meca.raw_tip()[2, 3]
        ty = 0.3e-3 * max(pen, 0.0) / 0.2 + rng.normal(0, 3e-5)
        return SimpleNamespace(force=[0, 0, 0], torque=[0, ty, 0], timestamp=next(ts))


stream = WrenchStream(Sensor())
det = ContactDetector(
    stream, ContactThresholds(torque_mnm=autocal.BACKSTOP_TORQUE_MNM, force_limit_n=autocal.BACKSTOP_FORCE_N)
)
view = LiveView("sim")
display = Display(view, Robot(meca), det, "microscope_cam", "sim")
s = autocal.Session(Robot(meca), display, bench, det)
home = np.array([-90, 40, 20, 90, 80, -110.0])


def fake_save(s, tip):  # no repo writes: just update the model tool in memory
    s.kin.tool[:3, 3] = tip


t0 = time.time()
autocal.stage_home_and_background(s, home)
first = autocal.stage_find_tip(s, home)
print("first contact true tip z", round(meca.true_tip()[2, 3], 3))
tip = autocal.stage_tip_calibration(s, first)
true_tip_frf = true_tool[:3, 3]
print(
    "RECOVERED tip",
    np.round(tip, 2).tolist(),
    " TRUE",
    np.round(true_tip_frf, 2).tolist(),
    " error",
    round(float(np.linalg.norm(tip - true_tip_frf)), 3),
    "mm",
)
fake_save(s, tip)
fit = autocal.stage_scope_calibration(s, first["rot"])
print("fit dish z", round(fit.surface_z_mm, 3), "true", TRUE_DISH_Z)
errs = autocal.stage_verify(s, fit, first["rot"])
print("verify errors px", np.round(errs, 2).tolist(), f" sim took {time.time() - t0:.0f} s")
stream.stop()
view.close()
