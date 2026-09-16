# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the Basler camera backend.

pypylon is not installed in CI, so these tests stand a fake pylon module in its place -
the same approach test_realsense.py takes with pyrealsense2, except the fake is built here
rather than replayed from an artifact.

The fake models the parts of the SDK whose *behaviour* the backend depends on, not just its
call signatures. In particular a mono camera's grab result really is a 2-D array and the
fake ImageFormatConverter really is what expands it to three channels, so the mono path is
exercised rather than assumed. That path is the one worth guarding: a single-channel frame
reaching the dataset writer is tagged as a depth map and silently written as TIFF/hevc.
"""

import time
from types import SimpleNamespace

import logging

import numpy as np
import pytest

from lerobot.cameras.basler.configuration_basler import BaslerCameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.utils.errors import DeviceNotConnectedError

SERIAL = "40012345"
SENSOR_WIDTH, SENSOR_HEIGHT = 2448, 2048

# A patch in the very corner of the sensor. It survives a downscale of the full sensor and
# is absent from a centred crop, which is what makes it evidence rather than decoration.
CORNER_RGB = (200, 100, 50)
BACKGROUND_RGB = (10, 20, 30)
CORNER_SIZE = 200


# --------------------------------------------------------------------------- fake pylon


class FakeGenericException(Exception):  # noqa: N818  - mirrors genicam.GenericException's name
    """Stands in for genicam.GenericException."""


class FakeParameter:
    """A GenICam node.

    `valid=False` models a pypylon >= 26.6 PlaceholderParameter: the node the camera does
    not have. Crucially it does *not* raise on access - it just refuses to do anything -
    which is exactly the behaviour that makes try/except probing useless on modern pypylon.
    """

    def __init__(self, value=None, minimum=None, maximum=None, valid=True, writable=True):
        self._value = value
        self.Min = minimum
        self.Max = maximum
        self._valid = valid
        self._writable = writable

    @property
    def Value(self):  # noqa: N802  - mirrors the pylon API
        if not self._valid:
            raise FakeGenericException("node is a placeholder")
        return self._value

    @Value.setter
    def Value(self, value):  # noqa: N802
        if not (self._valid and self._writable):
            raise FakeGenericException("node is not writable")
        self._value = value

    def IsValid(self):  # noqa: N802
        return self._valid

    def IsWritable(self):  # noqa: N802
        return self._valid and self._writable

    def SetValue(self, value, correction=None):  # noqa: N802
        if not (self._valid and self._writable):
            raise FakeGenericException("node is not writable")
        if self.Min is not None:
            value = max(self.Min, value)
        if self.Max is not None:
            value = min(self.Max, value)
        self._value = value

    def TrySetValue(self, value):  # noqa: N802
        if not (self._valid and self._writable):
            return False
        self._value = value
        return True

    def SetToMaximum(self):  # noqa: N802
        self.SetValue(self.Max)

    def SetToMinimum(self):  # noqa: N802
        self.SetValue(self.Min)

    def TrySetToMaximum(self):  # noqa: N802
        return self.TrySetValue(self.Max) if self.Max is not None else False

    def TrySetToMinimum(self):  # noqa: N802
        return self.TrySetValue(self.Min) if self.Min is not None else False

    def GetValueOrDefault(self, default):  # noqa: N802
        return self._value if self._valid else default

    def GetSymbolics(self):  # noqa: N802
        return []


class FakeGrabResult:
    """One retrieved frame, usable as a context manager like the real GrabResultPtr."""

    def __init__(self, array=None, valid=True, succeeded=True):
        self.Array = array
        self._valid = valid
        self._succeeded = succeeded
        self.ErrorCode = 0 if succeeded else 0xE1000014
        self.ErrorDescription = "" if succeeded else "buffer underrun"
        self.released = False

    def IsValid(self):  # noqa: N802
        return self._valid

    def GrabSucceeded(self):  # noqa: N802
        return self._succeeded

    def Release(self):  # noqa: N802
        self.released = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.Release()


class FakeInstantCamera:
    """A Basler device: nodes, open/close state, and a grab engine that hands out frames."""

    def __init__(
        self, device_info, sensor_frame, sfnc2=True, max_fps=30.0, exposure_us=10_000.0,
        resulting_rate=True,
    ):
        self.device_info = device_info
        self.sensor_frame = sensor_frame
        self._sfnc2 = sfnc2
        self._max_fps = max_fps

        self._open = False
        self._grabbing = False

        # Set to True to make the camera stop delivering frames without erroring, the way a
        # camera stuck waiting on a trigger behaves.
        self.stalled = False
        # Number of upcoming grabs to fail, as intermittent GigE packet loss does.
        self.grab_fails_remaining = 0

        height, width = sensor_frame.shape[:2]
        self.Width = FakeParameter(width, minimum=64, maximum=width)
        self.Height = FakeParameter(height, minimum=64, maximum=height)
        self.OffsetX = FakeParameter(0, minimum=0, maximum=0)
        self.OffsetY = FakeParameter(0, minimum=0, maximum=0)
        self.BinningHorizontal = FakeParameter(1, minimum=1, maximum=4)
        self.BinningVertical = FakeParameter(1, minimum=1, maximum=4)
        self.TriggerMode = FakeParameter("On")
        self.PixelFormat = FakeParameter("Mono8" if sensor_frame.ndim == 2 else "RGB8")
        self.MaxNumBuffer = FakeParameter(10)
        self.AcquisitionFrameRateEnable = FakeParameter(False)

        rate = FakeParameter(max_fps, minimum=1.0, maximum=max_fps)
        # Some models have no resulting-rate node at all (the acA5472-17ucMED is one), in
        # which case the camera must fall back to the exposure ceiling to judge the rate.
        resulting = FakeParameter(max_fps) if resulting_rate else FakeParameter(valid=False)
        # SFNC 2.x (USB3) and SFNC 1.x (GigE) name these differently; whichever set this
        # model does not speak is a placeholder, as it would be on real hardware.
        if sfnc2:
            self.AcquisitionFrameRate = rate
            self.ResultingAcquisitionFrameRate = resulting
            self.AcquisitionFrameRateAbs = FakeParameter(valid=False)
            self.ResultingFrameRateAbs = FakeParameter(valid=False)
        else:
            self.AcquisitionFrameRateAbs = rate
            self.ResultingFrameRateAbs = resulting
            self.AcquisitionFrameRate = FakeParameter(valid=False)
            self.ResultingAcquisitionFrameRate = FakeParameter(valid=False)

        # Exposure bounds the rate at 1/exposure whatever the link can carry, so it is the
        # fallback when the model reports no resulting-rate node. Read-only, as it is on a
        # real camera while ExposureAuto is running.
        exposure = FakeParameter(exposure_us, writable=False)
        if sfnc2:
            self.ExposureTime = exposure
            self.ExposureTimeAbs = FakeParameter(valid=False)
        else:
            self.ExposureTimeAbs = exposure
            self.ExposureTime = FakeParameter(valid=False)

    def Open(self):  # noqa: N802
        self._open = True

    def Close(self):  # noqa: N802
        self._open = False

    def IsOpen(self):  # noqa: N802
        return self._open

    def IsGrabbing(self):  # noqa: N802
        return self._grabbing

    def StartGrabbing(self, strategy):  # noqa: N802
        self._grabbing = True

    def StopGrabbing(self):  # noqa: N802
        self._grabbing = False

    def GetSfncVersion(self):  # noqa: N802
        return 2 if self._sfnc2 else 1

    def GetDeviceInfo(self):  # noqa: N802
        return self.device_info

    def RetrieveResult(self, timeout_ms, handling):  # noqa: N802
        if self.stalled:
            # Sleep a little so a stalled camera does not spin the read loop through its
            # whole failure budget in microseconds.
            time.sleep(0.15)
            return FakeGrabResult(valid=False)
        if self.grab_fails_remaining > 0:
            self.grab_fails_remaining -= 1
            return FakeGrabResult(array=None, valid=True, succeeded=False)
        return FakeGrabResult(array=self.sensor_frame.copy())


class FakeImageFormatConverter:
    """Converts a grab result to RGB8, replicating mono and passing colour through."""

    def __init__(self):
        self.OutputPixelFormat = None
        self.OutputBitAlignment = FakeParameter(None)
        self.MaxNumThreads = FakeParameter(1, minimum=1, maximum=8)

    def ImageHasDestinationFormat(self, grab_result):  # noqa: N802
        return grab_result.Array.ndim == 3

    def ConvertToArray(self, grab_result):  # noqa: N802
        array = grab_result.Array
        if array.ndim == 2:
            # What pylon does for Mono8 -> RGB8packed.
            return np.repeat(array[:, :, None], 3, axis=2)
        return array


def make_device_info(serial=SERIAL, model="acA2440-35um", user_name=None):
    return SimpleNamespace(
        GetSerialNumber=lambda: serial,
        GetModelName=lambda: model,
        GetFriendlyName=lambda: f"Basler {model} ({serial})",
        GetDeviceClass=lambda: "BaslerUsb",
        IsUserDefinedNameAvailable=lambda: user_name is not None,
        GetUserDefinedName=lambda: user_name,
    )


class FakeTlFactory:
    """The transport-layer singleton: enumerates devices and builds cameras from them."""

    def __init__(self):
        self.devices = []
        self.sensor_frame = None
        self.sfnc2 = True
        self.max_fps = 30.0
        self.open_raises = False
        self.exposure_us = 10_000.0
        self.resulting_rate = True

    def EnumerateDevices(self, filters=None):  # noqa: N802
        return list(self.devices)

    def CreateDevice(self, device_info):  # noqa: N802
        camera = FakeInstantCamera(
            device_info, self.sensor_frame, self.sfnc2, self.max_fps,
            self.exposure_us, self.resulting_rate,
        )
        if self.open_raises:
            camera.Open = lambda: (_ for _ in ()).throw(FakeGenericException("device is exclusively opened"))
        return camera

    CreateFirstDevice = CreateDevice


# ----------------------------------------------------------------------------- fixtures


def make_sensor_frame(mono: bool) -> np.ndarray:
    """A sensor image with a marked corner, mono or colour."""
    if mono:
        frame = np.full((SENSOR_HEIGHT, SENSOR_WIDTH), BACKGROUND_RGB[0], dtype=np.uint8)
        frame[:CORNER_SIZE, :CORNER_SIZE] = CORNER_RGB[0]
        return frame

    frame = np.zeros((SENSOR_HEIGHT, SENSOR_WIDTH, 3), dtype=np.uint8)
    frame[:, :] = BACKGROUND_RGB
    frame[:CORNER_SIZE, :CORNER_SIZE] = CORNER_RGB
    return frame


@pytest.fixture
def fake_pylon(monkeypatch):
    """Install the fake SDK into the camera module and return the factory to configure it."""
    from lerobot.cameras.basler import camera_basler
    from lerobot.utils import import_utils

    # require_package caches its answer, so seeding the cache is enough to convince the
    # constructor that pypylon is installed without touching the import machinery.
    monkeypatch.setitem(import_utils._require_package_cache, "pypylon", True)

    factory = FakeTlFactory()
    factory.devices = [make_device_info()]
    factory.sensor_frame = make_sensor_frame(mono=False)

    fake_pylon_module = SimpleNamespace(
        TlFactory=SimpleNamespace(GetInstance=lambda: factory),
        InstantCamera=lambda device: device,  # CreateDevice already returns the camera
        ImageFormatConverter=FakeImageFormatConverter,
        GrabStrategy_LatestImageOnly=0,
        TimeoutHandling_Return=1,
        PixelType_RGB8packed="RGB8packed",
        OutputBitAlignment_MsbAligned=0,
        FloatValueCorrection_ClipToRange=0,
        Sfnc_2_0_0=2,
    )
    fake_genicam_module = SimpleNamespace(GenericException=FakeGenericException)

    monkeypatch.setattr(camera_basler, "pylon", fake_pylon_module)
    monkeypatch.setattr(camera_basler, "genicam", fake_genicam_module)

    return factory


@pytest.fixture
def make_camera(fake_pylon):
    """Build a connected BaslerCamera, cleaning it up afterwards."""
    from lerobot.cameras.basler import BaslerCamera

    created = []

    def _make(connect=True, warmup=False, **config_kwargs):
        kwargs = {"serial_number": SERIAL, "fps": 30, "width": 640, "height": 480}
        kwargs.update(config_kwargs)
        camera = BaslerCamera(BaslerCameraConfig(**kwargs))
        created.append(camera)
        if connect:
            camera.connect(warmup=warmup)
        return camera

    yield _make

    for camera in created:
        camera.disconnect()


def read_one(camera, timeout_ms=2000):
    return camera.async_read(timeout_ms=timeout_ms)


# -------------------------------------------------------------------------------- config


def test_config_is_registered_as_basler():
    assert BaslerCameraConfig(serial_number=SERIAL).type == "basler"


def test_config_coerces_serial_to_string():
    assert BaslerCameraConfig(serial_number=40012345).serial_number == "40012345"


@pytest.mark.parametrize("binning", [0, -1, 1.5])
def test_config_rejects_bad_binning(binning):
    with pytest.raises(ValueError, match="binning"):
        BaslerCameraConfig(serial_number=SERIAL, binning=binning)


def test_config_rejects_half_specified_resolution():
    with pytest.raises(ValueError, match="must be set together"):
        BaslerCameraConfig(serial_number=SERIAL, width=640)


def test_factory_builds_a_basler_camera(fake_pylon):
    from lerobot.cameras.basler import BaslerCamera
    from lerobot.cameras.utils import make_cameras_from_configs

    cameras = make_cameras_from_configs({"microscope_cam": BaslerCameraConfig(serial_number=SERIAL)})
    assert isinstance(cameras["microscope_cam"], BaslerCamera)


# --------------------------------------------------------------------------- frame shape


@pytest.mark.parametrize("mono", [True, False], ids=["mono_sensor", "colour_sensor"])
def test_frames_are_three_channel_uint8(fake_pylon, make_camera, mono):
    """A mono sensor must not leak a 1-channel frame: that is read downstream as a depth map."""
    fake_pylon.sensor_frame = make_sensor_frame(mono=mono)
    camera = make_camera()

    frame = read_one(camera)

    assert frame.shape == (480, 640, 3)
    assert frame.dtype == np.uint8


def test_mono_frame_is_grey_not_garbage(fake_pylon, make_camera):
    """Replicating mono across RGB means every pixel's channels are equal."""
    fake_pylon.sensor_frame = make_sensor_frame(mono=True)
    camera = make_camera()

    frame = read_one(camera)

    assert np.array_equal(frame[:, :, 0], frame[:, :, 1])
    assert np.array_equal(frame[:, :, 1], frame[:, :, 2])


def test_colour_order_is_rgb_by_default(fake_pylon, make_camera):
    """Nothing downstream re-swaps channels, so RGB has to be right here."""
    camera = make_camera()

    frame = read_one(camera)

    assert tuple(frame[0, 0]) == CORNER_RGB


def test_colour_mode_bgr_swaps_channels(fake_pylon, make_camera):
    camera = make_camera(color_mode=ColorMode.BGR)

    frame = read_one(camera)

    assert tuple(frame[0, 0]) == CORNER_RGB[::-1]


# ------------------------------------------------------------------------ resize vs crop


def test_full_sensor_is_downscaled_not_cropped(fake_pylon, make_camera):
    """The whole field of view must survive.

    Writing Width/Height on a Basler crops the sensor. The corner patch is outside any
    centred 640x480 crop of a 2448x2048 sensor, so finding it in the output is proof the
    backend downscaled the full frame instead.
    """
    camera = make_camera()

    frame = read_one(camera)

    assert frame.shape[:2] == (480, 640)
    assert tuple(frame[0, 0]) == CORNER_RGB
    # ...and the rest of the sensor is still there too.
    assert tuple(frame[-1, -1]) == BACKGROUND_RGB


def test_sensor_roi_is_left_at_full_size(fake_pylon, make_camera):
    camera = make_camera()

    assert camera.capture_width == SENSOR_WIDTH
    assert camera.capture_height == SENSOR_HEIGHT


def test_resolution_defaults_to_the_sensor_size(fake_pylon, make_camera):
    camera = make_camera(width=None, height=None)

    assert (camera.width, camera.height) == (SENSOR_WIDTH, SENSOR_HEIGHT)
    assert read_one(camera).shape == (SENSOR_HEIGHT, SENSOR_WIDTH, 3)


# ------------------------------------------------------------------------------ rotation


def test_rotation_90_swaps_output_dimensions(fake_pylon, make_camera):
    """width/height stay the *post*-rotation dims, matching OpenCVCamera's contract."""
    camera = make_camera(rotation=Cv2Rotation.ROTATE_90)

    assert (camera.width, camera.height) == (640, 480)
    assert read_one(camera).shape == (480, 640, 3)


def test_rotation_180_keeps_dimensions_and_moves_the_corner(fake_pylon, make_camera):
    camera = make_camera(rotation=Cv2Rotation.ROTATE_180)

    frame = read_one(camera)

    assert frame.shape == (480, 640, 3)
    assert tuple(frame[-1, -1]) == CORNER_RGB


# ------------------------------------------------------------------------------ settings


def test_fps_is_applied_to_the_camera(fake_pylon, make_camera):
    camera = make_camera(fps=15)

    assert camera.camera.AcquisitionFrameRate.Value == 15.0
    # The rate node is ignored unless the enable flag is set first.
    assert camera.camera.AcquisitionFrameRateEnable.Value is True


def test_sfnc1_camera_uses_the_gige_parameter_names(fake_pylon, make_camera):
    """A GigE ace Classic speaks SFNC 1.x, where the rate node has a different name."""
    fake_pylon.sfnc2 = False
    camera = make_camera(fps=15)

    assert camera.camera.AcquisitionFrameRateAbs.Value == 15.0


def test_binning_is_applied_when_requested(fake_pylon, make_camera):
    camera = make_camera(binning=2)

    assert camera.camera.BinningHorizontal.Value == 2
    assert camera.camera.BinningVertical.Value == 2


def test_trigger_mode_is_turned_off(fake_pylon, make_camera):
    """A camera left in triggered mode delivers nothing, and only times out at warmup."""
    camera = make_camera()

    assert camera.camera.TriggerMode.Value == "Off"


def test_converter_targets_rgb8(fake_pylon, make_camera):
    camera = make_camera()

    assert camera.converter.OutputPixelFormat == "RGB8packed"


# ------------------------------------------------------------------------- read_latest


def test_read_latest_raises_before_the_first_frame(fake_pylon, make_camera):
    fake_pylon.sensor_frame = make_sensor_frame(mono=False)
    camera = make_camera(connect=False)
    camera.connect(warmup=False)
    camera.camera.stalled = True
    # Drop whatever the loop captured between connect() and the stall.
    with camera.frame_lock:
        camera.latest_frame = None
        camera.latest_timestamp = None

    with pytest.raises(RuntimeError, match="has not captured any frames yet"):
        camera.read_latest()


def test_read_latest_raises_when_the_frame_is_stale(fake_pylon, make_camera):
    camera = make_camera()
    read_one(camera)
    camera.camera.stalled = True
    time.sleep(0.05)

    with pytest.raises(TimeoutError, match="too old"):
        camera.read_latest(max_age_ms=1)


def test_read_latest_returns_a_fresh_frame_without_blocking(fake_pylon, make_camera):
    camera = make_camera()
    read_one(camera)

    start = time.perf_counter()
    frame = camera.read_latest(max_age_ms=5000)
    elapsed_ms = (time.perf_counter() - start) * 1e3

    assert frame.shape == (480, 640, 3)
    assert elapsed_ms < 50, "read_latest must not wait on hardware"


def test_read_latest_requires_a_connection(fake_pylon, make_camera):
    camera = make_camera(connect=False)

    with pytest.raises(DeviceNotConnectedError):
        camera.read_latest()


# -------------------------------------------------------------------- connect/disconnect


def test_connect_by_unknown_serial_raises_connection_error(fake_pylon, make_camera):
    """ConnectionError specifically: robots catch it to degrade when one camera is missing."""
    with pytest.raises(ConnectionError, match="No Basler camera with serial"):
        make_camera(serial_number="99999999")


def test_connect_error_lists_the_serials_that_were_found(fake_pylon, make_camera):
    with pytest.raises(ConnectionError, match=SERIAL):
        make_camera(serial_number="99999999")


def test_connect_with_no_devices_hints_at_the_usual_causes(fake_pylon, make_camera):
    fake_pylon.devices = []

    with pytest.raises(ConnectionError, match="pylon Viewer"):
        make_camera()


def test_connect_raises_connection_error_when_the_device_is_locked(fake_pylon, make_camera):
    fake_pylon.open_raises = True

    with pytest.raises(ConnectionError, match="could not open it"):
        make_camera()


def test_disconnect_after_a_failed_connect_is_a_noop(fake_pylon):
    """Robots tear cameras down unconditionally; raising here would abort the whole teardown."""
    from lerobot.cameras.basler import BaslerCamera

    camera = BaslerCamera(BaslerCameraConfig(serial_number="99999999"))
    with pytest.raises(ConnectionError):
        camera.connect(warmup=False)

    camera.disconnect()  # must not raise


def test_disconnect_closes_the_device_and_stops_the_thread(fake_pylon, make_camera):
    camera = make_camera()
    device = camera.camera

    camera.disconnect()

    assert not device.IsOpen()
    assert not device.IsGrabbing()
    assert camera.thread is None
    assert not camera.is_connected


def test_connect_warmup_waits_for_a_real_frame(fake_pylon, make_camera):
    camera = make_camera(connect=False)
    camera.connect(warmup=True)

    assert camera.latest_frame is not None


def test_reading_requires_a_connection(fake_pylon, make_camera):
    camera = make_camera(connect=False)

    with pytest.raises(DeviceNotConnectedError):
        camera.async_read()


def test_context_manager_connects_and_disconnects(fake_pylon):
    from lerobot.cameras.basler import BaslerCamera

    camera = BaslerCamera(BaslerCameraConfig(serial_number=SERIAL, fps=30, width=640, height=480))
    with camera:
        assert camera.is_connected
    assert not camera.is_connected


# ------------------------------------------------------------------------ error handling


def test_transient_failed_grabs_do_not_kill_the_read_thread(fake_pylon, make_camera):
    """GigE packet loss shows up as bursts of failed grabs; the camera should ride them out."""
    camera = make_camera()
    read_one(camera)

    # Fewer than the consecutive-failure budget, so the loop should recover on its own.
    camera.camera.grab_fails_remaining = 3

    assert read_one(camera).shape == (480, 640, 3)
    assert camera.thread is not None and camera.thread.is_alive()


@pytest.mark.filterwarnings("ignore::pytest.PytestUnhandledThreadExceptionWarning")
def test_sustained_failed_grabs_stop_the_read_thread(fake_pylon, make_camera):
    """A camera failing every grab is not coming back; the loop gives up rather than spin.

    Callers see this as read_latest()/async_read() raising "read thread is not running",
    which is the signal a robot needs to stop trusting this camera.
    """
    camera = make_camera()
    read_one(camera)

    camera.camera.grab_fails_remaining = 10_000

    deadline = time.perf_counter() + 5.0
    while camera.thread.is_alive() and time.perf_counter() < deadline:
        time.sleep(0.02)

    assert not camera.thread.is_alive()
    with pytest.raises(RuntimeError, match="read thread is not running"):
        camera.read_latest()


# ------------------------------------------------------------------------- find_cameras


def test_find_cameras_reports_serial_and_profile(fake_pylon):
    from lerobot.cameras.basler import BaslerCamera

    found = BaslerCamera.find_cameras()

    assert len(found) == 1
    assert found[0]["type"] == "Basler"
    assert found[0]["id"] == SERIAL
    assert found[0]["default_stream_profile"]["width"] == SENSOR_WIDTH


def test_find_cameras_survives_a_device_it_cannot_open(fake_pylon):
    from lerobot.cameras.basler import BaslerCamera

    fake_pylon.open_raises = True

    found = BaslerCamera.find_cameras()

    assert found[0]["id"] == SERIAL
    assert found[0]["default_stream_profile"] is None
    assert "could not open" in found[0]["error"]


def test_slow_exposure_is_warned_about_when_the_camera_reports_no_resulting_rate(
    fake_pylon, make_camera, caplog
):
    """A dim optical path must be diagnosed at connect, not as a timeout much later.

    Exposure bounds the frame rate at 1/exposure no matter what the link can carry, so a
    500 ms exposure caps the camera at 2 fps. On a model with no ResultingAcquisitionFrameRate
    node there is nothing else to check it against, and the first symptom used to be a
    TimeoutError out of async_read, which says nothing about exposure being the cause.
    """
    fake_pylon.resulting_rate = False
    fake_pylon.exposure_us = 500_000.0  # 500 ms -> 2 fps, as a dim objective produces

    with caplog.at_level(logging.WARNING, logger="lerobot.cameras.basler.camera_basler"):
        make_camera(fps=30)

    warnings = [r.message for r in caplog.records if r.levelname == "WARNING"]
    assert any("can only sustain 2.0 fps" in m for m in warnings), warnings
    assert any("Exposure is 500.0 ms" in m for m in warnings), warnings


def test_no_rate_warning_when_exposure_allows_the_requested_rate(fake_pylon, make_camera, caplog):
    """The fallback must not cry wolf: 10 ms allows 100 fps, comfortably above 30."""
    fake_pylon.resulting_rate = False
    fake_pylon.exposure_us = 10_000.0

    with caplog.at_level(logging.WARNING, logger="lerobot.cameras.basler.camera_basler"):
        make_camera(fps=30)

    assert not [r for r in caplog.records if "can only sustain" in r.message]
