"""Find the pipette tip in a microscope frame by background subtraction.

The Basler sees a bright circular field (the illuminated aperture) with a static but cluttered
background - scratches, particles, the dish's own features. The pipette is the only thing that
moves, so the reliable signal is the DIFFERENCE between a frame with the pipette in view and a
background frame taken with it out of view (or far out of focus):

    1. |frame - background| in grey, inside the illuminated circle only;
    2. threshold + morphological clean-up -> the pipette blob (largest connected component);
    3. the tip is the end of that blob furthest from where it enters the field of view - the
       blob's extreme point along its own principal axis, on the side away from the field edge.

Returns sub-pixel coordinates in the frame's own pixels (x right, y down), the convention the
scope calibration uses, so a detection can stand in for a human click.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class TipDetection:
    px: tuple[float, float]  # tip (u, v)
    axis: tuple[float, float]  # unit direction along the pipette, pointing at the tip
    blob_area: int  # pixels in the pipette blob
    contrast: float  # mean |difference| inside the blob, grey levels
    score: float  # 0..1 confidence heuristic (contrast and shape)


def field_mask(frame: np.ndarray, thresh: int = 40) -> np.ndarray:
    """Boolean mask of the illuminated circle (everything bright enough to see a pipette in)."""
    grey = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) if frame.ndim == 3 else frame
    mask = (grey > thresh).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return mask.astype(bool)
    keep = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = (lab == keep).astype(np.uint8)
    # shrink slightly: the bright rim of the aperture flickers with exposure
    return cv2.erode(mask, np.ones((9, 9), np.uint8)).astype(bool)


def detect_tip(
    frame: np.ndarray,
    background: np.ndarray,
    diff_thresh: float = 25.0,
    min_area: int = 150,
    min_tip_inset_px: float = 5.0,
    max_shank_inset_px: float = 30.0,
) -> TipDetection | None:
    """Tip of the pipette in ``frame`` (RGB uint8), given a ``background`` frame without it."""
    g1 = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY).astype(np.float32), (5, 5), 0)
    g0 = cv2.GaussianBlur(cv2.cvtColor(background, cv2.COLOR_RGB2GRAY).astype(np.float32), (5, 5), 0)
    fov = field_mask(background)
    # The Basler runs auto-exposure: a pipette entering the view darkens the image and the camera
    # brightens it back, shifting every pixel. Match the frames' brightness inside the field (by
    # the median, which the thin pipette barely moves) so only the pipette remains in the diff.
    m0, m1 = np.median(g0[fov]), np.median(g1[fov])
    if m1 > 1:
        g1 *= m0 / m1
    diff = np.abs(g1 - g0)
    diff[~fov] = 0
    blob = (diff > diff_thresh).astype(np.uint8)
    blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(blob)
    if n <= 1:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = int(stats[k, cv2.CC_STAT_AREA])
    if area < min_area:
        return None

    ys, xs = np.nonzero(lab == k)
    pts = np.stack([xs, ys], axis=1).astype(np.float64)
    centre = pts.mean(axis=0)
    c = pts - centre
    evals, evecs = np.linalg.eigh(c.T @ c)
    axis = evecs[:, -1]
    s = c @ axis

    # Which end is the tip? The pipette enters across the field's edge, so the end nearer the
    # edge of the illuminated circle is the shank; the other is the tip.
    # Distance to the nearest edge of the VISIBLE field. The image border counts as an edge:
    # distanceTransform only measures to zero pixels, and on Huanyu's setup the bright field runs
    # off the image, so without this a pipette entering at the image edge looked like a tip deep
    # inside the field (first arm run: "tip" at y = 639 of 640).
    fov_u8 = fov.astype(np.uint8)
    fov_u8[0, :] = fov_u8[-1, :] = fov_u8[:, 0] = fov_u8[:, -1] = 0
    dist = cv2.distanceTransform(fov_u8, cv2.DIST_L2, 5)
    end_hi = pts[s > s.max() - 3]
    end_lo = pts[s < s.min() + 3]
    d_hi = dist[end_hi[:, 1].astype(int), end_hi[:, 0].astype(int)].mean()
    d_lo = dist[end_lo[:, 1].astype(int), end_lo[:, 0].astype(int)].mean()
    if d_lo > d_hi:  # the low end is deeper inside the field -> it is the tip
        axis, s = -axis, -s
        d_lo, d_hi = d_hi, d_lo
    # A visible tip ends INSIDE the field while the shank runs out to its edge. If both ends sit
    # at the edge, the pipette is merely crossing the view with its tip outside it - reporting
    # the inner end as "the tip" would send a servo chasing the wrong point.
    if d_hi < min_tip_inset_px or d_lo > max_shank_inset_px:
        return None
    tip_pts = pts[s > s.max() - 2.0]
    tip = tip_pts.mean(axis=0)

    contrast = float(diff[lab == k].mean())
    elong = float(np.sqrt(evals[-1] / max(evals[0], 1e-9)))
    score = float(np.clip((contrast - diff_thresh) / 60.0, 0, 1) * np.clip(elong / 8.0, 0, 1))
    return TipDetection(
        px=(float(tip[0]), float(tip[1])),
        axis=(float(axis[0]), float(axis[1])),
        blob_area=area,
        contrast=contrast,
        score=score,
    )


def draw_detection(frame: np.ndarray, det: TipDetection | None) -> np.ndarray:
    """Copy of ``frame`` with the detected tip (magenta cross) and pipette axis drawn."""
    out = frame.copy()
    if det is None:
        return out
    u, v = det.px
    cv2.drawMarker(out, (int(round(u)), int(round(v))), (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
    a = np.asarray(det.axis)
    p0 = (int(u - 60 * a[0]), int(v - 60 * a[1]))
    cv2.line(out, p0, (int(u), int(v)), (255, 0, 255), 1)
    return out
