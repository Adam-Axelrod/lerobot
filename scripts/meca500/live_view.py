"""A live image window with key presses, mouse clicks and markers, built on matplotlib.

The project pins ``opencv-python-headless`` (see pyproject.toml), so ``cv2.imshow`` /
``cv2.namedWindow`` are unavailable - the same reason camera_tool.py previews with matplotlib.
Pixel coordinates are the image's own (x right, y down, pixel centres at integers), the same
convention cv2 uses, so clicked points can be used directly as image pixels.

matplotlib's default shortcuts are switched off for the window's lifetime, because several
collide with the calibration keys (s = save figure dialog, q = close, f = fullscreen, c/v =
back/forward ...). The toolbar's zoom/pan still work; clicks made while one of those tools is
active are ignored, so you can zoom in to click a tip precisely.

    view = LiveView("calibrate_scope")
    while not view.closed:
        view.show(frame, title="...", points=[(u, v, "red")], crosshair=True)
        key = view.key()        # None, or the key pressed since the last call
        click = view.click()    # None, or (u, v) of a left click on the image
"""

from __future__ import annotations

from collections import deque

import matplotlib.pyplot as plt
import numpy as np


class LiveView:
    def __init__(self, title: str, figsize=(6.5, 9.0)):
        self._saved_keymaps = {k: v for k, v in plt.rcParams.items() if k.startswith("keymap.")}
        for k in self._saved_keymaps:
            plt.rcParams[k] = []
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.fig.canvas.manager.set_window_title(title)
        self.ax.axis("off")
        self.fig.tight_layout()
        self._img = None
        self._artists: list = []
        self._keys: deque[str] = deque()
        self._clicks: deque[tuple[float, float]] = deque()
        self.closed = False
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("close_event", lambda _e: setattr(self, "closed", True))
        plt.show(block=False)

    def _on_key(self, event) -> None:
        if event.key:
            self._keys.append(event.key)

    def _on_click(self, event) -> None:
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar is not None and getattr(toolbar, "mode", ""):
            return  # zooming/panning, not picking
        if event.button == 1 and event.inaxes is self.ax and event.xdata is not None:
            self._clicks.append((float(event.xdata), float(event.ydata)))

    def show(self, frame: np.ndarray, title: str = "", points=(), crosshair: bool = False) -> None:
        """Draw ``frame`` (HxWx3 RGB) with optional markers ``(u, v, colour[, style])``."""
        if self.closed:
            return
        if self._img is None:
            self._img = self.ax.imshow(frame, interpolation="nearest")
        else:
            self._img.set_data(frame)
        for a in self._artists:
            a.remove()
        self._artists = []
        h, w = frame.shape[:2]
        if crosshair:
            self._artists += self.ax.plot([w / 2 - 20, w / 2 + 20], [h / 2, h / 2], color="yellow", lw=1)
            self._artists += self.ax.plot([w / 2, w / 2], [h / 2 - 20, h / 2 + 20], color="yellow", lw=1)
        for p in points:
            u, v, colour = p[:3]
            style = p[3] if len(p) > 3 else "o"
            self._artists += self.ax.plot(
                [u], [v], style, color=colour, ms=6, mfc="none" if style == "+" else colour
            )
        self.ax.set_title(title, fontsize=9)
        plt.pause(0.001)

    def key(self) -> str | None:
        return self._keys.popleft() if self._keys else None

    def click(self) -> tuple[float, float] | None:
        return self._clicks.popleft() if self._clicks else None

    def close(self) -> None:
        plt.close(self.fig)
        plt.rcParams.update(self._saved_keymaps)
