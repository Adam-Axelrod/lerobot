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

"""Provides the BaslerCamera class for capturing frames from Basler cameras via pypylon.

Written against the pypylon >= 26.6 API, which changed in ways that silently break older
code. The two that matter here:

  * Accessing a node that does not exist no longer raises - it returns a
    ``PlaceholderParameter`` whose ``IsValid()``/``IsWritable()`` are False. The classic
    "wrap the probe in try/except GenericException" idiom therefore stops firing, and every
    optional feature below uses the ``TrySetValue``/``TrySetToMaximum``/``GetValueOrDefault``
    family instead.
  * ``ImageFormatConverter.OutputPixelFormat`` is a plain attribute, not a parameter node,
    so it takes a bare assignment rather than ``.Value``.

``PylonInitialize()``/``PylonTerminate()`` are called by pypylon itself at import and at
interpreter shutdown, so this module must never call them.
"""

import logging
import time
from threading import Event, Lock, Thread
from typing import TYPE_CHECKING, Any

import cv2  # type: ignore  # TODO: add type stubs for OpenCV
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import _pypylon_available, require_package

from ..camera import Camera
from ..utils import get_cv2_rotation
from .configuration_basler import BaslerCameraConfig, ColorMode

if TYPE_CHECKING or _pypylon_available:
    from pypylon import genicam, pylon  # type: ignore  # TODO: pypylon ships no type stubs
else:
    genicam = None  # type: ignore[assignment]
    pylon = None  # type: ignore[assignment]

logger = logging.getLogger(__name__)

# How long a single RetrieveResult waits before reporting no frame. Only reached when the
# camera has genuinely stopped delivering, since the grab strategy keeps a frame queued.
GRAB_TIMEOUT_MS = 1000

# Buffers in the grab engine's pool. The pylon default is 10; a handful is plenty when the
# output queue is capped at 1, and fewer buffers means less memory held per camera.
MAX_NUM_BUFFER = 5

# Consecutive background-read failures tolerated before the read thread gives up, matching
# OpenCVCamera. A camera that fails this many times in a row is not coming back on its own.
MAX_READ_FAILURES = 10


class BaslerCamera(Camera):
    """Manages frame capture from a Basler camera through the pylon SDK.

    Cameras are addressed by serial number, never by enumeration index: pylon's own docs
    warn that index order changes with the number of connected devices. Find the serial with:

    ```bash
    python scripts/meca500/basler_probe.py --list
    ```

    Frames are always delivered as `(height, width, 3)` uint8 RGB (or BGR, if configured).
    Monochrome and colour cameras both work with no configuration: every grab goes through a
    `pylon.ImageFormatConverter` targeting `RGB8packed`, which replicates Mono8 to three
    channels and demosaics Bayer, so the caller never sees the difference.

    Example:
        ```python
        from lerobot.cameras.basler import BaslerCamera, BaslerCameraConfig

        config = BaslerCameraConfig(serial_number="40012345", fps=30, width=640, height=480)
        camera = BaslerCamera(config)
        camera.connect()

        frame = camera.read()                       # blocking, waits for a fresh frame
        frame = camera.async_read()                 # latest unconsumed frame
        frame = camera.read_latest(max_age_ms=200)  # non-blocking peek

        camera.disconnect()
        ```
    """

    def __init__(self, config: BaslerCameraConfig):
        """Initializes the BaslerCamera instance.

        Args:
            config: The configuration settings for the camera.
        """
        require_package("pypylon", extra="basler", import_name="pypylon")
        super().__init__(config)

        self.config = config
        self.serial_number = config.serial_number

        self.fps = config.fps
        self.color_mode = config.color_mode
        self.binning = config.binning
        self.warmup_s = config.warmup_s

        self.camera: Any | None = None  # pylon.InstantCamera
        self.converter: Any | None = None  # pylon.ImageFormatConverter

        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.latest_frame: NDArray[Any] | None = None
        self.latest_timestamp: float | None = None
        self.new_frame_event: Event = Event()

        self.rotation: int | None = get_cv2_rotation(config.rotation)

        # Raw sensor dimensions, discovered at connect().
        self.capture_width: int | None = None
        self.capture_height: int | None = None
        self._reset_connection_settings()

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number})"

    def _reset_connection_settings(self) -> None:
        """Restore settings that may have been auto-detected during a failed connection."""
        self.fps = self.config.fps
        self.width = self.config.width
        self.height = self.config.height
        self.capture_width, self.capture_height = None, None

        # width/height are the *post-rotation* output dims, so the resize that happens before
        # rotation targets the swapped pair for a quarter turn. Same contract as OpenCVCamera's
        # capture_width/capture_height, except here the swap applies to the resize target
        # rather than to what we ask the device for (the device always gives us full sensor).
        self._resize_width, self._resize_height = self.width, self.height
        if self.rotation in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
            self._resize_width, self._resize_height = self.height, self.width

    @property
    def is_connected(self) -> bool:
        """Checks if the camera is currently connected and opened."""
        if self.camera is None:
            return False
        try:
            return bool(self.camera.IsOpen())
        except genicam.GenericException:
            # The device vanished mid-session (unplug, network loss). Report disconnected
            # rather than propagating: callers poll this every control step.
            return False

    @check_if_already_connected
    def connect(self, warmup: bool = True) -> None:
        """Connects to the Basler camera with the configured serial number.

        Opens the device, applies the capture settings, starts grabbing with
        `GrabStrategy_LatestImageOnly`, and spins up the background read thread.

        Args:
            warmup: If True, waits at connect() time until at least one valid frame has been
                captured by the background thread. Defaults to True.

        Raises:
            DeviceAlreadyConnectedError: If the camera is already connected.
            ConnectionError: If no camera with this serial number is found, or it cannot be
                opened or configured. Always ConnectionError rather than a bare RuntimeError,
                because robots (e.g. Meca500.connect) catch exactly that to degrade
                gracefully when one camera of several is unavailable.
        """
        self.camera = self._open_device()

        try:
            self._configure_capture_settings()
            self._setup_converter()

            self.camera.MaxNumBuffer.Value = MAX_NUM_BUFFER
            # LatestImageOnly caps the output queue at 1: each new frame replaces the queued
            # one, so RetrieveResult never hands back a backlog. This is the driver-side
            # equivalent of read_latest(), and is what keeps the control loop from paying for
            # frames it already skipped.
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            self._start_read_thread()

            if warmup and self.warmup_s > 0:
                start_time = time.time()
                while time.time() - start_time < self.warmup_s:
                    self.async_read(timeout_ms=self.warmup_s * 1000)
                    time.sleep(0.1)
                with self.frame_lock:
                    if self.latest_frame is None:
                        raise ConnectionError(f"{self} failed to capture frames during warmup.")
        except BaseException as e:
            try:
                self._cleanup_resources()
            except Exception:
                logger.exception(f"Failed to fully clean up {self} after connect() failed.")
            self._reset_connection_settings()
            if isinstance(e, genicam.GenericException):
                raise ConnectionError(f"{self} failed to configure: {e}") from e
            raise

        logger.info(f"{self} connected.")

    def _open_device(self) -> Any:
        """Find the camera by serial number and open it.

        Enumerates explicitly rather than letting `CreateFirstDevice` do the matching, so a
        missing camera produces an error naming the serials that *were* found - by far the
        most useful thing to know when this fails.
        """
        tlf = pylon.TlFactory.GetInstance()
        try:
            devices = list(tlf.EnumerateDevices())
        except genicam.GenericException as e:
            raise ConnectionError(f"Could not enumerate Basler devices: {e}") from e

        matches = [d for d in devices if d.GetSerialNumber() == self.serial_number]
        if not matches:
            found = [f"{d.GetSerialNumber()} ({d.GetModelName()})" for d in devices] or ["none"]
            raise ConnectionError(
                f"No Basler camera with serial number {self.serial_number!r}. Found: {', '.join(found)}. "
                f"If nothing was found at all: close the pylon Viewer (it holds an exclusive lock), and "
                f"check that the pylon Camera Software Suite is installed - the pip wheel ships user-space "
                f"runtimes only, and USB3 cameras need the kernel driver from the installer to enumerate. "
                f"Run 'python scripts/meca500/basler_probe.py --list' to debug."
            )

        try:
            camera = pylon.InstantCamera(tlf.CreateDevice(matches[0]))
            camera.Open()
        except genicam.GenericException as e:
            raise ConnectionError(
                f"Found Basler camera {self.serial_number} but could not open it: {e}. "
                f"It is most likely still open in the pylon Viewer or another process."
            ) from e

        return camera

    @check_if_not_connected
    def _configure_capture_settings(self) -> None:
        """Applies binning, full-sensor ROI and frame rate to the opened camera.

        Must run between `Open()` and `StartGrabbing()`: binning and ROI are not writable
        while the camera is grabbing.

        Every optional feature goes through the `Try*` family rather than a bare write.
        Feature sets vary widely across Basler models, and under pypylon >= 26.6 a missing
        node returns a placeholder instead of raising, so a plain assignment to a node this
        model lacks would fail in a far less obvious way.
        """
        if self.camera is None:
            raise DeviceNotConnectedError(f"{self} camera is not initialized")

        cam = self.camera

        # Free-running acquisition. Without this a camera left in triggered mode by a previous
        # session delivers nothing, and the only symptom is a warmup timeout.
        cam.TriggerMode.TrySetValue("Off")

        if self.binning > 1:
            applied = cam.BinningHorizontal.TrySetValue(self.binning)
            applied &= cam.BinningVertical.TrySetValue(self.binning)
            if not applied:
                # Binning is asked for to fit the link's bandwidth. Silently not applying it
                # means the frame rate quietly stays where it was, with nothing to explain it.
                logger.warning(
                    f"{self} does not support binning={self.binning}; it was NOT applied and the "
                    f"full-resolution bandwidth still applies."
                )

        # Full sensor. Offsets shrink first: with a large ROI still in place there is no room
        # to grow Width/Height, and the writes would clip to the existing window.
        cam.OffsetX.TrySetToMinimum()
        cam.OffsetY.TrySetToMinimum()
        cam.Width.TrySetToMaximum()
        cam.Height.TrySetToMaximum()

        self.capture_width = int(cam.Width.Value)
        self.capture_height = int(cam.Height.Value)

        # No requested output size: take the sensor's own, which becomes the dataset feature
        # shape. Both must be concrete ints before a robot reads observation_features.
        if self.width is None or self.height is None:
            self.width, self.height = self.capture_width, self.capture_height
            self._resize_width, self._resize_height = self.capture_width, self.capture_height
            if self.rotation in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE):
                self.width, self.height = self.capture_height, self.capture_width

        # SFNC version, not the transport layer, is the real split between the two parameter
        # vocabularies: SFNC 2.x (USB3 ace/ace 2/dart/boost) vs SFNC 1.x (GigE ace Classic).
        sfnc2 = cam.GetSfncVersion() >= pylon.Sfnc_2_0_0

        if self.fps is not None:
            # The rate cap is ignored unless explicitly enabled first.
            cam.AcquisitionFrameRateEnable.TrySetValue(True)
            node = cam.AcquisitionFrameRate if sfnc2 else cam.AcquisitionFrameRateAbs
            if node.IsValid() and node.IsWritable():
                node.SetValue(float(self.fps), pylon.FloatValueCorrection_ClipToRange)
            else:
                logger.warning(f"{self} does not support setting the frame rate; using its own default.")

        resulting_node = cam.ResultingAcquisitionFrameRate if sfnc2 else cam.ResultingFrameRateAbs
        resulting_fps = resulting_node.GetValueOrDefault(float("nan"))

        logger.info(
            f"{self} sensor {self.capture_width}x{self.capture_height} "
            f"(binning={self.binning}, pixel format={cam.PixelFormat.GetValueOrDefault('unknown')}) "
            f"-> output {self.width}x{self.height} at {resulting_fps:.1f} fps (requested {self.fps})."
        )

        # A camera that cannot sustain the requested rate starves the control loop, and the
        # only other symptom is read_latest() falling back to async_read() on every step.
        # `resulting_fps == resulting_fps` filters the NaN the node returns when unsupported.
        if self.fps is not None and resulting_fps == resulting_fps and resulting_fps < self.fps * 0.95:
            logger.warning(
                f"{self} can only sustain {resulting_fps:.1f} fps, below the requested {self.fps} fps. "
                f"Reduce the exposure time, or enable binning to cut the bandwidth per frame."
            )

    def _setup_converter(self) -> None:
        """Builds the RGB8 output converter.

        Targeting RGB8packed unconditionally is what makes mono-vs-colour a non-decision:
        pylon replicates Mono8 across three channels and demosaics Bayer, so both sensor kinds
        arrive as (H, W, 3) uint8 through one code path. Emitting a single-channel frame
        instead would be quietly destructive - `hw_to_dataset_features` tags any 1-channel
        feature as a *depth map* and routes it to TIFF/hevc encoding.
        """
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_RGB8packed  # plain attribute, not .Value
        converter.OutputBitAlignment.Value = pylon.OutputBitAlignment_MsbAligned
        converter.MaxNumThreads.SetToMaximum()
        self.converter = converter

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Detects available Basler cameras connected to the system.

        Returns:
            A list of dicts with 'name', 'type', 'id' (the serial number) and the device's
            default stream profile.
        """
        require_package("pypylon", extra="basler", import_name="pypylon")

        tlf = pylon.TlFactory.GetInstance()
        found_cameras_info = []

        for device in tlf.EnumerateDevices():
            # Unset DeviceInfo properties return a sentinel string rather than raising, so the
            # availability predicate is the only way to tell "unset" from a real value.
            user_name = device.GetUserDefinedName() if device.IsUserDefinedNameAvailable() else None

            info: dict[str, Any] = {
                "name": device.GetFriendlyName(),
                "type": "Basler",
                "id": device.GetSerialNumber(),
                "model": device.GetModelName(),
                "device_class": device.GetDeviceClass(),
                "user_defined_name": user_name,
            }

            # Opening the device is the only way to read its geometry, and it fails if the
            # camera is in use. Enumeration data is still worth returning in that case.
            try:
                camera = pylon.InstantCamera(tlf.CreateDevice(device))
                camera.Open()
                try:
                    sfnc2 = camera.GetSfncVersion() >= pylon.Sfnc_2_0_0
                    fps_node = camera.ResultingAcquisitionFrameRate if sfnc2 else camera.ResultingFrameRateAbs
                    info["default_stream_profile"] = {
                        "format": camera.PixelFormat.GetValueOrDefault("unknown"),
                        "width": int(camera.Width.Value),
                        "height": int(camera.Height.Value),
                        "fps": fps_node.GetValueOrDefault(float("nan")),
                    }
                finally:
                    camera.Close()
            except genicam.GenericException as e:
                info["default_stream_profile"] = None
                info["error"] = f"could not open device: {e}"

            found_cameras_info.append(info)

        return found_cameras_info

    def _read_from_hardware(self) -> NDArray[Any]:
        """Retrieve one frame from the grab engine as an (H, W, 3) uint8 RGB array."""
        if self.camera is None or self.converter is None:
            raise DeviceNotConnectedError(f"{self} camera is not initialized")

        # TimeoutHandling_Return rather than ThrowException: an idle camera is an ordinary
        # condition on this path, and IsValid() is cheaper than unwinding an exception.
        with self.camera.RetrieveResult(GRAB_TIMEOUT_MS, pylon.TimeoutHandling_Return) as grab_result:
            if not grab_result.IsValid():
                raise TimeoutError(f"{self} received no frame within {GRAB_TIMEOUT_MS} ms.")

            if not grab_result.GrabSucceeded():
                raise RuntimeError(
                    f"{self} grab failed (code={grab_result.ErrorCode:#x}): {grab_result.ErrorDescription}"
                )

            if self.converter.ImageHasDestinationFormat(grab_result):
                # Already RGB8: .Array is a copy of the buffer, so it stays valid after the
                # grab result is released on the way out of this block. Never publish
                # GetArrayZeroCopy()/GetMemoryView() instead - those views die with it.
                return grab_result.Array

            # ConvertToArray writes into a freshly allocated array (no intermediate
            # PylonImage), which is the documented choice for a live grab loop.
            return self.converter.ConvertToArray(grab_result)

    @check_if_not_connected
    def read(self) -> NDArray[Any]:
        """Reads a single frame, waiting for a fresh one.

        Does not touch the hardware directly: the background thread owns the device, so this
        clears the new-frame flag and waits for the next frame that thread publishes.

        Returns:
            The captured frame as (height, width, 3), in the configured color mode and with
            any configured rotation applied.

        Raises:
            DeviceNotConnectedError: If the camera is not connected.
            RuntimeError: If the background read thread is not running.
            TimeoutError: If no frame arrives within 10 seconds.
        """
        start_time = time.perf_counter()

        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")

        self.new_frame_event.clear()
        frame = self.async_read(timeout_ms=10000)

        read_duration_ms = (time.perf_counter() - start_time) * 1e3
        logger.debug(f"{self} read took: {read_duration_ms:.1f}ms")

        return frame

    def _postprocess_image(self, image: NDArray[Any]) -> NDArray[Any]:
        """Downscales, colour-converts and rotates a raw RGB frame.

        Args:
            image: The converter's output, expected (H, W, 3) uint8 RGB at sensor resolution.

        Returns:
            The processed frame at the configured output size.

        Raises:
            ValueError: If the configured `color_mode` is invalid.
            RuntimeError: If the frame's dimensions or channel count are unexpected.
        """
        if self.color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"Invalid color mode '{self.color_mode}'. Expected {ColorMode.RGB} or {ColorMode.BGR}."
            )

        if image.ndim != 3:
            raise RuntimeError(
                f"{self} frame has shape {image.shape}, expected 3 dimensions (H, W, C). "
                f"The pylon converter should have expanded mono frames to 3 channels."
            )

        h, w, c = image.shape

        if c != 3:
            raise RuntimeError(f"{self} frame channels={c} do not match expected 3 channels (RGB/BGR).")

        if h != self.capture_height or w != self.capture_width:
            raise RuntimeError(
                f"{self} frame width={w} or height={h} do not match the sensor's "
                f"width={self.capture_width} or height={self.capture_height}."
            )

        processed_image = image

        # Downscale the full sensor to the requested output size. Writing Width/Height on the
        # device would crop instead, losing everything outside the window - on a microscope,
        # most of the field of view. INTER_AREA is the correct filter for shrinking.
        if (w, h) != (self._resize_width, self._resize_height):
            processed_image = cv2.resize(
                processed_image,
                (self._resize_width, self._resize_height),
                interpolation=cv2.INTER_AREA,
            )

        # The converter emits RGB, so RGB is the no-op path here - the opposite of
        # OpenCVCamera, whose backend hands it BGR.
        if self.color_mode == ColorMode.BGR:
            processed_image = cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR)

        if self.rotation in (cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180):
            processed_image = cv2.rotate(processed_image, self.rotation)

        return processed_image

    def _restart_grabbing(self) -> None:
        """Stop and restart acquisition after an error.

        pylon's own guidance: once a grab error surfaces, the buffer/pipeline state is no
        longer trustworthy, so recover by restarting acquisition rather than by retrying the
        failed retrieve.
        """
        if self.camera is None:
            return
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def _read_loop(self) -> None:
        """Internal loop run by the background thread for asynchronous reading.

        On each iteration: grab a frame, post-process it, publish it under the lock and signal
        the new-frame event. Stops on DeviceNotConnectedError; other errors are logged and
        retried up to `MAX_READ_FAILURES` consecutive times.
        """
        stop_event = self.stop_event
        if stop_event is None:
            raise RuntimeError(f"{self}: stop_event is not initialized before starting read loop.")

        failure_count = 0
        while not stop_event.is_set():
            try:
                raw_frame = self._read_from_hardware()
                processed_frame = self._postprocess_image(raw_frame)
                capture_time = time.perf_counter()

                with self.frame_lock:
                    self.latest_frame = processed_frame
                    self.latest_timestamp = capture_time
                self.new_frame_event.set()
                failure_count = 0

            except DeviceNotConnectedError:
                break
            except Exception as e:
                if stop_event.is_set():
                    # Racing with disconnect(): the device was closed underneath us, which is
                    # the expected way this loop ends, not a failure worth reporting.
                    break
                if failure_count < MAX_READ_FAILURES:
                    failure_count += 1
                    logger.warning(f"Error reading frame in background thread for {self}: {e}")
                    if isinstance(e, genicam.GenericException):
                        try:
                            self._restart_grabbing()
                        except genicam.GenericException:
                            logger.exception(f"{self} failed to restart grabbing.")
                            break
                else:
                    raise RuntimeError(f"{self} exceeded maximum consecutive read failures.") from e

    def _start_read_thread(self) -> None:
        """Starts or restarts the background read thread if it's not running."""
        self._stop_read_thread()

        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(), name=f"{self}_read_loop")
        self.thread.daemon = True
        self.thread.start()
        time.sleep(0.1)

    def _stop_read_thread(self) -> None:
        """Signals the background read thread to stop and waits for it to join."""
        if self.stop_event is not None:
            self.stop_event.set()

        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                logger.warning(f"{self} read thread did not terminate within timeout.")

        self.thread = None
        self.stop_event = None

        with self.frame_lock:
            self.latest_frame = None
            self.latest_timestamp = None
            self.new_frame_event.clear()

    def _cleanup_resources(self) -> None:
        """Stop background reads and close the device, including after partial setup."""
        read_thread = self.thread
        camera = self.camera

        try:
            self._stop_read_thread()
        finally:
            self.camera = None
            self.converter = None
            try:
                if camera is not None:
                    try:
                        if camera.IsGrabbing():
                            camera.StopGrabbing()
                    except genicam.GenericException:
                        logger.exception(f"{self} failed to stop grabbing.")
                    try:
                        if camera.IsOpen():
                            camera.Close()
                    except genicam.GenericException:
                        logger.exception(f"{self} failed to close the device.")
            finally:
                # Closing the device may unblock a hardware read that outlived the first
                # bounded join in _stop_read_thread().
                if read_thread is not None and read_thread.is_alive():
                    read_thread.join(timeout=2.0)
                    if read_thread.is_alive():  # pragma: no cover
                        logger.warning(f"{self} read thread remained alive after closing the device.")

    @check_if_not_connected
    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        """Returns the latest frame the background thread has not yet handed out.

        Blocks only if the buffer is empty or its frame was already consumed by a previous
        call, so in a control loop this synchronises to the camera's frame rate.

        Args:
            timeout_ms: Maximum time in milliseconds to wait for a new frame. Defaults to 200.

        Returns:
            The latest captured frame as (height, width, 3).

        Raises:
            DeviceNotConnectedError: If the camera is not connected.
            TimeoutError: If no new frame arrives within the timeout.
            RuntimeError: If the background read thread is not running.
        """
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")

        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(
                f"Timed out waiting for frame from camera {self} after {timeout_ms} ms. "
                f"Read thread alive: {self.thread.is_alive()}."
            )

        with self.frame_lock:
            frame = self.latest_frame
            self.new_frame_event.clear()

        if frame is None:
            raise RuntimeError(f"Internal error: Event set but no frame available for {self}.")

        return frame

    @check_if_not_connected
    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        """Return the most recent frame captured immediately (Peeking).

        Non-blocking: returns whatever is in the buffer, which may already be stale. This is
        the method robot control loops call every step, so it must never wait on hardware.

        Returns:
            The frame image (numpy array).

        Raises:
            TimeoutError: If the latest frame is older than `max_age_ms`.
            DeviceNotConnectedError: If the camera is not connected.
            RuntimeError: If the camera is connected but has not captured any frames yet.
        """
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")

        with self.frame_lock:
            frame = self.latest_frame
            timestamp = self.latest_timestamp

        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any frames yet.")

        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(
                f"{self} latest frame is too old: {age_ms:.1f} ms (max allowed: {max_age_ms} ms)."
            )

        return frame

    def disconnect(self) -> None:
        """Disconnects from the camera and releases the device.

        Unlike OpenCVCamera, disconnecting an already-disconnected camera is a no-op rather
        than an error. Robots tear their cameras down unconditionally (`Meca500.disconnect`
        loops over every camera without a guard), so a camera whose connect() failed would
        otherwise abort the whole teardown - taking the arm's shutdown sequence with it.
        """
        if not self.is_connected and self.thread is None:
            logger.debug(f"{self} is already disconnected; nothing to do.")
            return

        self._cleanup_resources()

        logger.info(f"{self} disconnected.")
