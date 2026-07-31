"""Overlay a recorded dataset frame on the LIVE camera feed to check your desk setup.

Before a rollout, the scene has to be laid out the way it was when the dataset was
recorded — the microscope, the pipette holder, the dish, the lighting. A policy that
was trained on one layout will happily drive the arm into a slightly-moved microscope.

This script grabs ONE frame from a LeRobot dataset (Hugging Face Hub or local cache)
and blends it over that same camera's live picture, so you can nudge things on the
desk until the two line up. Run it ON THE WINDOWS LAPTOP (that's where the cameras
are), edit the CONFIG block, then:

    python scripts/meca500/check_positioning.py

The live camera is opened from Meca500MicroscopeConfig — same resolution, focus,
exposure and (on Windows) the same resolve-by-device-name logic as recording and
rollout — so what you see here is what the policy will see.

Controls (live matplotlib window):
    m           cycle overlay mode: blend -> diff -> checker -> edges -> split
    up/down     alpha: more dataset frame / more live feed (also the split position)
    n / p       next / previous dataset frame
    N / P       jump 10 frames
    e           cycle episode (reloads the dataset for that episode)
    q / esc     quit

Overlay modes:
    blend    straight alpha blend — the classic "ghost" overlay
    diff     |dataset - live| as a heatmap; black = aligned, bright = something moved
    checker  alternating tiles of dataset / live; edges break across tiles if misaligned
    edges    dataset edges (Canny) drawn in magenta over the live feed — best for
             lining up hard shapes like the microscope barrel or a dish rim
    split    vertical wipe: dataset on the left, live on the right, alpha moves the seam
"""

import platform

import cv2  # type: ignore
import matplotlib.pyplot as plt
import numpy as np

from lerobot.cameras.opencv import OpenCVCamera
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.meca500_microscope.config_meca500_microscope import Meca500MicroscopeConfig

# ----------------------------- CONFIG -----------------------------
# Dataset to compare against: a Hugging Face Hub repo_id (downloaded + cached on
# first run), or a local dataset already under HF_LEROBOT_HOME.
DATASET_REPO_ID = "AdamAxelrod/microscope_pipette"

# Which camera to check. A substring is enough — "overhead", "wrist", "microscope"
# all match both the dataset key (observation.images.overhead_cam) and the robot
# config camera name (overhead_cam).
CAMERA = "overhead"

# Which frame to overlay. EPISODE is the episode index; FRAME is the frame number
# *within* that episode (0 = the first frame, i.e. the reset/start-of-episode
# layout, which is usually the one you want to match). Both are adjustable live
# with n/p/e, so these are just the starting point.
EPISODE = 0
FRAME = 0

# Starting blend: 1.0 = only the dataset frame, 0.0 = only the live feed.
ALPHA = 0.5

# Starting overlay mode — one of MODES below.
MODE = "blend"

# Video decoder for the dataset's camera streams. "pyav" bundles its own FFmpeg
# and works out of the box; torchcodec needs FFmpeg DLLs that aren't installed
# on this machine (same reason train_microscope.py pins pyav).
VIDEO_BACKEND = "pyav"

# Local dataset root. None = the normal HF cache (HF_LEROBOT_HOME).
DATASET_ROOT: str | None = None
# ------------------------------------------------------------------

MODES = ["blend", "diff", "checker", "edges", "split"]
CHECKER_TILE = 60  # px, tile size for the checkerboard mode
EDGE_COLOR = (255, 0, 255)  # magenta, in RGB — rare in the scene, so edges pop


def match_key(candidates: list[str], wanted: str, what: str) -> str:
    """Pick the one candidate containing `wanted` (case-insensitive), or explain why not."""
    hits = [c for c in candidates if wanted.lower() in c.lower()]
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise ValueError(f"CAMERA='{wanted}' matches no {what}. Available: {candidates}")
    raise ValueError(f"CAMERA='{wanted}' is ambiguous — matches {hits}. Be more specific.")


def to_uint8_rgb(frame) -> np.ndarray:
    """Dataset image tensor (CHW, float in [0,1] or uint8) -> HWC uint8 RGB array."""
    arr = frame.numpy() if hasattr(frame, "numpy") else np.asarray(frame)
    if arr.ndim == 3 and arr.shape[0] in (1, 3):  # CHW -> HWC
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        arr = (np.clip(arr, 0.0, 1.0) * 255.0).astype(np.uint8)
    if arr.shape[2] == 1:
        arr = np.repeat(arr, 3, axis=2)
    return np.ascontiguousarray(arr)


def load_dataset(episode: int) -> LeRobotDataset:
    """Load the dataset restricted to one episode, so frame indices are episode-relative."""
    return LeRobotDataset(
        DATASET_REPO_ID,
        root=DATASET_ROOT,
        episodes=[episode],
        video_backend=VIDEO_BACKEND,
    )


def reference_frame(dataset: LeRobotDataset, key: str, frame: int) -> np.ndarray:
    """The dataset frame to overlay, as HWC uint8 RGB."""
    return to_uint8_rgb(dataset[frame % len(dataset)][key])


def blend(ref: np.ndarray, live: np.ndarray, alpha: float) -> np.ndarray:
    return cv2.addWeighted(ref, alpha, live, 1.0 - alpha, 0.0)


def difference(ref: np.ndarray, live: np.ndarray) -> np.ndarray:
    """Absolute difference as a heatmap — black where the scenes agree."""
    diff = cv2.absdiff(cv2.cvtColor(ref, cv2.COLOR_RGB2GRAY), cv2.cvtColor(live, cv2.COLOR_RGB2GRAY))
    return cv2.cvtColor(cv2.applyColorMap(diff, cv2.COLORMAP_INFERNO), cv2.COLOR_BGR2RGB)


def checkerboard(ref: np.ndarray, live: np.ndarray, tile: int = CHECKER_TILE) -> np.ndarray:
    """Alternating tiles of each image — misalignment shows as edges jumping at tile borders."""
    h, w = ref.shape[:2]
    ys, xs = np.mgrid[0:h, 0:w]
    mask = (((ys // tile) + (xs // tile)) % 2).astype(bool)
    out = live.copy()
    out[mask] = ref[mask]
    return out


def edge_overlay(ref: np.ndarray, live: np.ndarray, alpha: float) -> np.ndarray:
    """Canny edges of the dataset frame painted over the live feed."""
    edges = cv2.Canny(cv2.cvtColor(ref, cv2.COLOR_RGB2GRAY), 80, 160)
    edges = cv2.dilate(edges, np.ones((2, 2), np.uint8))
    out = live.copy()
    # alpha fades the edge lines so they can be checked against the pixels underneath
    out[edges > 0] = (
        alpha * np.array(EDGE_COLOR) + (1.0 - alpha) * out[edges > 0]
    ).astype(np.uint8)
    return out


def split_view(ref: np.ndarray, live: np.ndarray, alpha: float) -> np.ndarray:
    """Vertical wipe: dataset on the left of the seam, live on the right."""
    w = ref.shape[1]
    seam = int(np.clip(alpha, 0.0, 1.0) * w)
    out = live.copy()
    out[:, :seam] = ref[:, :seam]
    if 0 < seam < w:
        out[:, max(seam - 1, 0) : seam + 1] = EDGE_COLOR
    return out


def compose(mode: str, ref: np.ndarray, live: np.ndarray, alpha: float) -> np.ndarray:
    if mode == "diff":
        return difference(ref, live)
    if mode == "checker":
        return checkerboard(ref, live)
    if mode == "edges":
        return edge_overlay(ref, live, alpha)
    if mode == "split":
        return split_view(ref, live, alpha)
    return blend(ref, live, alpha)


def main() -> None:
    robot_cfg = Meca500MicroscopeConfig(id="meca500_microscope")
    cam_name = match_key(list(robot_cfg.cameras), CAMERA, "camera in Meca500MicroscopeConfig")

    print(f"Loading episode {EPISODE} of {DATASET_REPO_ID} (first run downloads it)...")
    dataset = load_dataset(EPISODE)
    ds_key = match_key(dataset.meta.camera_keys, CAMERA, "camera in the dataset")

    state = {"episode": EPISODE, "frame": FRAME, "alpha": ALPHA, "mode": MODE, "stop": False}
    if state["mode"] not in MODES:
        raise ValueError(f"MODE='{MODE}' is not one of {MODES}")
    ref = reference_frame(dataset, ds_key, state["frame"])

    camera = OpenCVCamera(robot_cfg.cameras[cam_name])
    print(f"Opening live camera '{cam_name}' (index {robot_cfg.cameras[cam_name].index_or_path})...")
    camera.connect()

    print(f"  dataset key : {ds_key}  ({dataset.num_frames} frames in episode {EPISODE})")
    print(f"  dataset res : {ref.shape[1]}x{ref.shape[0]}")
    print("  keys: m=mode  up/down=alpha  n/p=frame  N/P=+-10  e=episode  q=quit")
    if platform.system() != "Windows":
        print("  NOTE: not on Windows — camera indices are NOT resolved by device name.")

    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 7))
    fig.canvas.manager.set_window_title(f"positioning check — {cam_name} vs {DATASET_REPO_ID}")
    ax.axis("off")
    im = None

    def reload_reference() -> None:
        nonlocal ref
        ref = reference_frame(dataset, ds_key, state["frame"])

    def on_key(event) -> None:
        nonlocal dataset
        key = event.key
        if key in ("q", "escape"):
            state["stop"] = True
        elif key == "m":
            state["mode"] = MODES[(MODES.index(state["mode"]) + 1) % len(MODES)]
        elif key in ("up", "right"):
            state["alpha"] = min(1.0, state["alpha"] + 0.05)
        elif key in ("down", "left"):
            state["alpha"] = max(0.0, state["alpha"] - 0.05)
        elif key in ("n", "p", "N", "P"):
            step = {"n": 1, "p": -1, "N": 10, "P": -10}[key]
            state["frame"] = (state["frame"] + step) % dataset.num_frames
            reload_reference()
        elif key == "e":
            state["episode"] = (state["episode"] + 1) % dataset.meta.total_episodes
            print(f"[episode] loading episode {state['episode']}...")
            dataset = load_dataset(state["episode"])
            state["frame"] = min(state["frame"], dataset.num_frames - 1)
            reload_reference()

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda _evt: state.update(stop=True))

    try:
        while not state["stop"]:
            live = camera.read()
            if live.shape[:2] != ref.shape[:2]:
                # Dataset and live resolutions should match (same config), but a dataset
                # recorded at another resolution still lines up once rescaled.
                live = cv2.resize(live, (ref.shape[1], ref.shape[0]), interpolation=cv2.INTER_AREA)
            frame = compose(state["mode"], ref, live, state["alpha"])
            if im is None:
                im = ax.imshow(frame)
            else:
                im.set_data(frame)
            ax.set_title(
                f"{cam_name}  |  mode={state['mode']}  alpha={state['alpha']:.2f}  "
                f"ep={state['episode']} frame={state['frame']}/{dataset.num_frames - 1}\n"
                "m=mode  up/down=alpha  n/p=frame  N/P=+-10  e=episode  q=quit",
                fontsize=10,
            )
            plt.pause(0.001)
    finally:
        camera.disconnect()
        plt.close(fig)


if __name__ == "__main__":
    main()
