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


def image_shift(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    """Sub-pixel translation (dx, dy) of frame ``b`` relative to ``a`` (RGB uint8).

    Measured on the dish's fine texture (specks, scratches) with the illumination's slow falloff
    removed, so a pipette in one frame barely moves it. A background that has shifted from the
    live frame turns every speck into a difference blob, and blobs next to the tip join it: in E1
    (2026-11-04) backgrounds 3-11 px off sent near-checkpoint corrections 150-280 um the wrong way.
    """
    g = [cv2.cvtColor(x, cv2.COLOR_RGB2GRAY).astype(np.float32) for x in (a, b)]
    g = [x - cv2.GaussianBlur(x, (0, 0), 8) for x in g]
    (dx, dy), _ = cv2.phaseCorrelate(g[0], g[1], cv2.createHanningWindow(g[0].shape[::-1], cv2.CV_32F))
    return float(dx), float(dy)


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


# "The tip" for aiming and scoring is the centre of the pipette OPENING, not the end of the glass
# (Adam, 2026-11-12: blind by-eye verdicts on 53 final frames followed the target sitting on the
# opening; the detected glass end did not predict them, AUC 0.56). In the frame the opening is the
# bright spot just behind the dark end wall, about 6-8 px (80-100 um) back along the pipette from
# detect_tip's point. That point stays the physical end of the glass, which is what touches the dish
# (contact detection, calibration touches, the tool model).
OPENING_SEARCH_PX = 14.0  # look this far back from the glass end
OPENING_MIN_CONTRAST = 20.0  # grey levels between the end wall and the opening, else fall back
OPENING_FALLBACK_PX = 7.0  # median offset when the opening cannot be seen (blur, glare)
OPENING_PEAK_PX = 4.0  # the opening's peak lies within this of its half-height rise
OPENING_ACROSS_PX = 12.0  # search this far either side of the glass end's line for the lumen (pipette ~30 px wide)


def find_opening(frame: np.ndarray, det: TipDetection) -> tuple[tuple[float, float], bool]:
    """Centre of the pipette opening behind ``det.px``, and whether it was seen (else the fallback).

    The lumen is a bright stripe inside the dark glass, and it need not lie on the line through the
    glass end (2-3 px to one side on bench 2). So first find the stripe across the pipette: the
    brightness peak across, averaged 4-12 px behind the end. Then sample along the stripe from the
    glass end backwards, averaged over 3 px across: the end wall is a dark minimum within 5 px of the
    end, and the opening's centre is the brightest point within OPENING_PEAK_PX of where the
    brightness first rises halfway from the wall to the maximum (behind it the lumen can stay bright,
    so the whole bright run is not the opening)."""
    g = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY).astype(np.float32), (3, 3), 0)
    a = np.asarray(det.axis, float)
    n = np.array([-a[1], a[0]])
    tip = np.asarray(det.px, float)
    sample = lambda pts: np.array([cv2.getRectSubPix(g, (1, 1), tuple(q.astype(np.float32)))[0, 0] for q in pts])
    step = 0.25
    # across: where is the lumen stripe?
    ws = np.arange(-OPENING_ACROSS_PX, OPENING_ACROSS_PX + 1e-9, step)
    across = np.array([sample([tip - t * a + w * n for t in np.arange(4.0, 12.01, 1.0)]).mean() for w in ws])
    # The lumen is a RIDGE: brighter than the glass on both sides within 4 px. A maximum at the
    # window's edge is the bright dish beyond the pipette, not the lumen.
    k4 = int(4.0 / step)
    best, w_c = 0.0, 0.0
    for i in range(k4, len(ws) - k4):
        if across[i] < across[i - 1] or across[i] < across[i + 1]:
            continue
        prom = across[i] - max(across[i - k4 : i].min(), across[i + 1 : i + k4 + 1].min())
        if prom > best:
            best, w_c = prom, float(ws[i])
            y0, y1, y2 = across[i - 1], across[i], across[i + 1]
            den = y0 - 2 * y1 + y2
            if den < 0:
                w_c += step * 0.5 * (y0 - y2) / den
    if best < OPENING_MIN_CONTRAST * 0.5:
        w_c = 0.0
    base = tip + w_c * n
    # along the stripe
    ts = np.arange(0.0, OPENING_SEARCH_PX + 1e-9, step)
    prof = np.array([sample([base - t * a + w * n for w in (-1.5, 0.0, 1.5)]).mean() for t in ts])
    fallback = (tuple(base - OPENING_FALLBACK_PX * a), False)
    i_wall = int(np.argmin(prof[ts <= 5.0]))
    lo, hi = prof[i_wall], prof[i_wall:].max()
    if hi - lo < OPENING_MIN_CONTRAST:
        return fallback
    i_up = i_wall + int(np.argmax(prof[i_wall:] >= lo + 0.5 * (hi - lo)))
    win = prof[i_up : i_up + int(OPENING_PEAK_PX / step) + 1]
    i_pk = i_up + int(np.argmax(win))
    t_c = ts[i_pk]
    if 0 < i_pk < len(prof) - 1:  # parabolic sub-sample peak
        y0, y1, y2 = prof[i_pk - 1], prof[i_pk], prof[i_pk + 1]
        den = y0 - 2 * y1 + y2
        if den < 0:
            t_c += step * 0.5 * (y0 - y2) / den
    if not 2.0 <= t_c <= OPENING_SEARCH_PX - 1.0:
        return fallback
    return tuple(base - t_c * a), True


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
