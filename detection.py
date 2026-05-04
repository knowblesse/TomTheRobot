"""
detection.py
Per-frame blob detection.

Two targets:
  1. RED MARKER on the robot — a colored sticker/cap. Detected by sampling
     the marker's color from a clicked region, then thresholding in LAB
     color space. LAB's a* channel separates red from white background
     more robustly than HSV under uneven lighting.

  2. DARK HOOD of a Long-Evans rat — the head/shoulders pigmented region.
     Detected by thresholding the V channel (HSV) below a chosen value.
     Works because the arena floor is white.

The detector is stateless. It takes a frame and parameters, returns a
single best Detection (or None). Identity / tracking / occlusion handling
lives in tracker.py, not here.

Calibration helpers are also here: they're interactive (use cv.selectROI
or a custom polygon picker) and produce the parameters the detectors need.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2 as cv
import numpy as np


# -----------------------------------------------------------------
#                          Data types
# -----------------------------------------------------------------

@dataclass
class Detection:
    """One detected blob."""
    x: float                 # centroid x (pixels)
    y: float                 # centroid y (pixels)
    area: float              # contour area (pixels^2)
    contour: np.ndarray      # for debug overlay


@dataclass
class RedMarkerParams:
    """Parameters for detecting the red robot marker."""
    target_lab: np.ndarray   # shape (3,), float; sampled at calibration
    color_range: int         # +/- around target in each LAB channel
    min_area: int
    max_area: int


@dataclass
class RatHoodParams:
    """Parameters for detecting the dark hood of a Long-Evans rat."""
    v_threshold: int         # pixels with V <= this are "dark"
    min_area: int
    max_area: int


# -----------------------------------------------------------------
#                       Helper utilities
# -----------------------------------------------------------------

def _kernel(size: int) -> np.ndarray:
    """Elliptical structuring element of given size; size must be odd >=1."""
    s = max(1, int(size))
    return cv.getStructuringElement(cv.MORPH_ELLIPSE, (s, s))


def _denoise_mask(mask: np.ndarray,
                  open_size: int = 3,
                  close_size: int = 15) -> np.ndarray:
    """Open (kill speckle) then close (fill holes). Returns a new mask."""
    m = cv.morphologyEx(mask, cv.MORPH_OPEN, _kernel(open_size))
    m = cv.morphologyEx(m, cv.MORPH_CLOSE, _kernel(close_size))
    return m


def _largest_contour_in_range(
    mask: np.ndarray,
    min_area: int,
    max_area: int,
) -> Optional[np.ndarray]:
    """Find the largest contour whose area is in [min_area, max_area]."""
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    best = None
    best_area = -1.0
    for c in cnts:
        a = cv.contourArea(c)
        if a < min_area or a > max_area:
            continue
        if a > best_area:
            best = c
            best_area = a
    return best


def _centroid(contour: np.ndarray) -> Tuple[float, float]:
    """Centroid via image moments. Fallback to bounding-circle center if degenerate."""
    M = cv.moments(contour)
    if M["m00"] > 1e-6:
        return M["m10"] / M["m00"], M["m01"] / M["m00"]
    (cx, cy), _ = cv.minEnclosingCircle(contour)
    return float(cx), float(cy)


def _apply_mask(frame_bgr: np.ndarray, mask: Optional[np.ndarray]) -> np.ndarray:
    """Apply optional global ROI mask. Returns a copy, original is unchanged."""
    if mask is None:
        return frame_bgr
    return cv.bitwise_and(frame_bgr, frame_bgr, mask=mask)


def polygon_to_mask(
    polygon: List[Tuple[int, int]],
    frame_size: Tuple[int, int],
) -> np.ndarray:
    """Rasterize a polygon to a binary mask of given size.

    Args:
        polygon: list of (x, y) vertex tuples
        frame_size: (height, width) — same convention as np.ndarray.shape[:2]

    Returns:
        uint8 mask, 0 outside the polygon, 255 inside.
        If fewer than 3 vertices, returns a fully-on mask.
    """
    h, w = frame_size
    mask = np.zeros((h, w), dtype=np.uint8)
    if len(polygon) < 3:
        mask[:] = 255
        return mask
    pts = np.array(polygon, dtype=np.int32).reshape(-1, 1, 2)
    cv.fillPoly(mask, [pts], 255)
    return mask


# -----------------------------------------------------------------
#                    Detection: red robot marker
# -----------------------------------------------------------------

def detect_red_marker(
    frame_bgr: np.ndarray,
    params: RedMarkerParams,
    global_mask: Optional[np.ndarray] = None,
) -> Optional[Detection]:
    """Find the red robot marker. Returns the largest valid blob, or None."""
    img = _apply_mask(frame_bgr, global_mask)
    lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)

    target = params.target_lab.astype(np.float32)
    r = float(params.color_range)
    lo = np.clip(target - r, 0, 255).astype(np.uint8)
    hi = np.clip(target + r, 0, 255).astype(np.uint8)

    mask = cv.inRange(lab, lo, hi)
    mask = _denoise_mask(mask)

    cnt = _largest_contour_in_range(mask, params.min_area, params.max_area)
    if cnt is None:
        return None
    cx, cy = _centroid(cnt)
    return Detection(x=cx, y=cy, area=float(cv.contourArea(cnt)), contour=cnt)


# -----------------------------------------------------------------
#                Detection: rat hood (dark on white)
# -----------------------------------------------------------------

def detect_rat_hood(
    frame_bgr: np.ndarray,
    params: RatHoodParams,
    global_mask: Optional[np.ndarray] = None,
) -> Optional[Detection]:
    """Find the rat's dark hood. Returns the largest valid blob, or None.

    Uses the V (value) channel of HSV. Dark pixels (V <= threshold) become
    foreground. Robust to lighting variation in *hue* and *saturation*,
    which we don't care about for a black-on-white scene.
    """
    img = _apply_mask(frame_bgr, global_mask)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    v = hsv[:, :, 2]

    # The global_mask sets pixels outside the arena to (0,0,0), which would
    # also satisfy V<=threshold and produce huge spurious "dark" regions.
    # Re-mask after thresholding so only inside-arena pixels qualify.
    dark = (v <= params.v_threshold).astype(np.uint8) * 255
    if global_mask is not None:
        dark = cv.bitwise_and(dark, global_mask)

    dark = _denoise_mask(dark, open_size=3, close_size=11)

    cnt = _largest_contour_in_range(dark, params.min_area, params.max_area)
    if cnt is None:
        return None
    cx, cy = _centroid(cnt)
    return Detection(x=cx, y=cy, area=float(cv.contourArea(cnt)), contour=cnt)


# -----------------------------------------------------------------
#                Polygon mask picker (interactive)
# -----------------------------------------------------------------

def calibrate_polygon_mask(
    frame_bgr: np.ndarray,
    window_name: str = "Select arena polygon",
) -> Tuple[List[Tuple[int, int]], np.ndarray]:
    """Interactive polygon picker.

    Controls:
        Left click           add a vertex
        Right click / 'z'    remove the last vertex
        ENTER / SPACE        finish (need >=3 points); polygon auto-closes
        ESC / 'q'            cancel; returns full-frame mask
        'r'                  reset (clear all points)

    Returns:
        (polygon, mask) where polygon is a list of (x, y) tuples and
        mask is a uint8 array of frame_bgr's (height, width).
        If the user cancels, polygon is [] and mask is all-on.
    """
    h, w = frame_bgr.shape[:2]
    points: List[Tuple[int, int]] = []
    cursor = [0, 0]

    def on_mouse(event, x, y, flags, _userdata):
        cursor[0], cursor[1] = x, y
        if event == cv.EVENT_LBUTTONDOWN:
            points.append((x, y))
        elif event == cv.EVENT_RBUTTONDOWN:
            if points:
                points.pop()

    cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
    cv.setMouseCallback(window_name, on_mouse)

    HELP_LINES = [
        "Left click: add vertex     Right click / Z: undo",
        "ENTER: finish              ESC: cancel             R: reset",
    ]

    while True:
        disp = frame_bgr.copy()

        # Draw polygon-in-progress
        if len(points) >= 1:
            for i, p in enumerate(points):
                cv.circle(disp, p, 4, (0, 255, 255), -1)
                cv.putText(disp, str(i + 1), (p[0] + 6, p[1] - 6),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 0, 0), 3, cv.LINE_AA)
                cv.putText(disp, str(i + 1), (p[0] + 6, p[1] - 6),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5,
                           (0, 255, 255), 1, cv.LINE_AA)
            for a, b in zip(points, points[1:]):
                cv.line(disp, a, b, (0, 255, 255), 1, cv.LINE_AA)
            # rubber-band line from last vertex to current cursor
            cv.line(disp, points[-1], (cursor[0], cursor[1]),
                    (0, 200, 200), 1, cv.LINE_AA)
            # closing preview if we have >=3 points
            if len(points) >= 3:
                cv.line(disp, (cursor[0], cursor[1]), points[0],
                        (0, 200, 200), 1, cv.LINE_AA)

        # Header text (with black outline for legibility)
        for i, line in enumerate(HELP_LINES):
            org = (10, 25 + 22 * i)
            cv.putText(disp, line, org, cv.FONT_HERSHEY_SIMPLEX, 0.55,
                       (0, 0, 0), 3, cv.LINE_AA)
            cv.putText(disp, line, org, cv.FONT_HERSHEY_SIMPLEX, 0.55,
                       (255, 255, 255), 1, cv.LINE_AA)
        status = f"vertices: {len(points)}"
        cv.putText(disp, status, (10, h - 12),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv.LINE_AA)
        cv.putText(disp, status, (10, h - 12),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv.LINE_AA)

        cv.imshow(window_name, disp)
        key = cv.waitKey(20) & 0xFF

        if key in (13, 32):  # ENTER or SPACE
            if len(points) >= 3:
                break
        elif key in (27, ord("q")):
            cv.destroyWindow(window_name)
            full = np.full((h, w), 255, dtype=np.uint8)
            return [], full
        elif key == ord("z"):
            if points:
                points.pop()
        elif key == ord("r"):
            points.clear()

    cv.destroyWindow(window_name)
    mask = polygon_to_mask(points, (h, w))
    return points, mask


# -----------------------------------------------------------------
#               Color / threshold sampling helpers
# -----------------------------------------------------------------

def calibrate_red_marker_color(
    frame_bgr: np.ndarray,
    window_name: str = "Select red marker",
) -> Optional[np.ndarray]:
    """Drag a rectangle over the red marker; return median LAB color (3,).
    Returns None if the user cancels (no selection / ESC)."""
    annotated = frame_bgr.copy()
    cv.putText(annotated, "Drag a rectangle over the red marker, ENTER",
               (10, 30), cv.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2, cv.LINE_AA)
    x, y, w, h = cv.selectROI(window_name, annotated,
                              showCrosshair=False, fromCenter=False)
    cv.destroyWindow(window_name)
    if w <= 0 or h <= 0:
        print("[calib] red marker calibration cancelled")
        return None

    region = frame_bgr[y:y+h, x:x+w]
    lab_region = cv.cvtColor(region, cv.COLOR_BGR2LAB)
    pixels = lab_region.reshape(-1, 3)
    median = np.median(pixels, axis=0)
    print(f"[calib] Red marker LAB median = {median.astype(int).tolist()}")
    return median.astype(np.float32)


def calibrate_rat_threshold(
    frame_bgr: np.ndarray,
    window_name: str = "Select rat hood region",
) -> Optional[int]:
    """Drag a rectangle over a piece of the rat's dark hood; return a V
    threshold computed as ~95th percentile of the selected pixels' V values.
    Returns None if the user cancels (no selection / ESC)."""
    annotated = frame_bgr.copy()
    cv.putText(annotated, "Drag a rectangle over the rat's dark hood, ENTER",
               (10, 30), cv.FONT_HERSHEY_DUPLEX, 0.7, (0, 255, 255), 2, cv.LINE_AA)
    x, y, w, h = cv.selectROI(window_name, annotated,
                              showCrosshair=False, fromCenter=False)
    cv.destroyWindow(window_name)
    if w <= 0 or h <= 0:
        print("[calib] rat threshold calibration cancelled")
        return None

    region = frame_bgr[y:y+h, x:x+w]
    hsv = cv.cvtColor(region, cv.COLOR_BGR2HSV)
    v = hsv[:, :, 2].ravel()
    thr = int(np.percentile(v, 95))
    print(f"[calib] Rat V threshold = {thr} (region 95th percentile)")
    return thr
