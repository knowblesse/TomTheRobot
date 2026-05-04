"""
tools/test_detection.py
Standalone interactive tester for color/blob detection.

Where values come from:
  - All numeric defaults (color range, min/max area, V threshold) live
    in config.py. Edit config.py to change defaults.
  - The arena polygon is the only thing persisted to JSON
    (./last_calibration.json) and auto-loaded on startup.
  - The red color sample and the rat darkness threshold must be sampled
    each session via 'c' and 'r' (they depend on the day's lighting).

Workflow (first time):
  1. Run the script. The webcam opens.
  2. Press 'm' to draw the arena polygon (click vertices, ENTER to finish).
  3. Press 'w' to save the polygon. Next run will auto-load it.
  4. Press 'c' to sample the RED MARKER color (drag a box over it).
  5. Press 'r' to sample the RAT HOOD darkness.
  6. Tune live with the keys shown at the bottom of the window.
     Tuned values are transient — to make them permanent, tell Claude
     and the new defaults will be folded into config.py.

Usage (from project root):
    python -m tools.test_detection
    python -m tools.test_detection --index 1
    python -m tools.test_detection --calib custom.json
"""

import argparse
import sys
import time

import cv2 as cv
import numpy as np

sys.path.insert(0, ".")

import config
from calibration_io import SessionCalibration, load_calibration, save_calibration
from capture import FrameGrabber
from detection import (
    RatHoodParams,
    RedMarkerParams,
    calibrate_polygon_mask,
    calibrate_rat_threshold,
    calibrate_red_marker_color,
    detect_rat_hood,
    detect_red_marker,
    polygon_to_mask,
)


# Colors for overlays (BGR)
COLOR_RED = (0, 0, 255)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)

DEFAULT_CALIB_PATH = "./last_calibration.json"


def draw_text(img, text, org, color=COLOR_YELLOW, scale=0.55):
    """Two-pass text with black outline for legibility on busy backgrounds."""
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               COLOR_BLACK, 3, cv.LINE_AA)
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               color, 1, cv.LINE_AA)


def draw_detection(img, det, color, label):
    """Draw centroid marker, contour outline, and label."""
    if det is None:
        return
    cx, cy = int(det.x), int(det.y)
    cv.drawContours(img, [det.contour], -1, color, 2)
    cv.drawMarker(img, (cx, cy), color,
                  markerType=cv.MARKER_CROSS, markerSize=18, thickness=2)
    draw_text(img, f"{label} ({cx},{cy}) area={int(det.area)}",
              (cx + 12, cy - 8), color=color)


def draw_polygon_outline(img, polygon):
    """Draw the arena polygon outline on the display."""
    if len(polygon) < 2:
        return
    pts = np.array(polygon, dtype=np.int32).reshape(-1, 1, 2)
    cv.polylines(img, [pts], isClosed=True, color=(0, 200, 200),
                 thickness=1, lineType=cv.LINE_AA)


def get_freshest_blocking(grabber, timeout_s=2.0):
    """Wait briefly for a fresh frame, used during interactive prompts."""
    t0 = time.monotonic()
    last_idx = -1
    while time.monotonic() - t0 < timeout_s:
        f = grabber.get_latest()
        if f is not None and f.frame_idx != last_idx:
            return f
        time.sleep(0.01)
    return grabber.get_latest()


def make_red_params(target_lab: np.ndarray) -> RedMarkerParams:
    """Build red params with config defaults for the numeric fields."""
    return RedMarkerParams(
        target_lab=target_lab,
        color_range=config.ROBOT_COLOR_RANGE,
        min_area=config.ROBOT_MIN_AREA_PX,
        max_area=config.ROBOT_MAX_AREA_PX,
    )


def make_rat_params(v_threshold: int) -> RatHoodParams:
    """Build rat params with config defaults for the numeric fields."""
    return RatHoodParams(
        v_threshold=v_threshold,
        min_area=config.RAT_MIN_AREA_PX,
        max_area=config.RAT_MAX_AREA_PX,
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, default=config.CAMERA_INDEX)
    p.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    p.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    p.add_argument("--fps", type=int, default=config.CAMERA_FPS)
    p.add_argument("--calib", type=str, default=DEFAULT_CALIB_PATH)
    args = p.parse_args()

    grabber = FrameGrabber(
        camera_index=args.index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=config.CAMERA_FOURCC,
    )
    try:
        grabber.open()
    except Exception as e:
        print(f"[error] {e}")
        return 1
    grabber.start()

    win = "test_detection"
    cv.namedWindow(win, cv.WINDOW_AUTOSIZE)

    # ---- Try to load polygon, red sample, rat threshold from disk ----
    polygon = []
    global_mask = None
    red_params = None
    rat_params = None

    try:
        loaded = load_calibration(args.calib)
    except ValueError as e:
        print(f"[calib] {e}")
        loaded = None

    if loaded is not None:
        polygon = list(loaded.arena_polygon)
        if polygon:
            global_mask = polygon_to_mask(polygon, loaded.frame_size)
        if loaded.red_target_lab is not None:
            red_params = make_red_params(
                np.array(loaded.red_target_lab, dtype=np.float32)
            )
            print(f"[test] red sample restored: LAB={list(loaded.red_target_lab)}")
        if loaded.rat_v_threshold is not None:
            rat_params = make_rat_params(int(loaded.rat_v_threshold))
            print(f"[test] rat threshold restored: V<={loaded.rat_v_threshold}")

    if global_mask is None:
        print("[test] no saved polygon; press 'm' to set arena.")
    if red_params is None:
        print("[test] no saved red sample; press 'c' to sample.")
    if rat_params is None:
        print("[test] no saved rat threshold; press 'r' to sample.")

    last_frame_idx = -1
    save_idx = 0
    last_red = None
    last_rat = None

    try:
        while True:
            frame = grabber.get_latest()
            if frame is None:
                if cv.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                continue

            new_frame = (frame.frame_idx != last_frame_idx)
            last_frame_idx = frame.frame_idx

            disp = frame.image.copy()

            # Dim everything outside the arena polygon
            if global_mask is not None:
                inv = cv.bitwise_not(global_mask)
                outside = cv.bitwise_and(disp, disp, mask=inv)
                outside = (outside * 0.3).astype(np.uint8)
                inside = cv.bitwise_and(disp, disp, mask=global_mask)
                disp = cv.add(inside, outside)
                draw_polygon_outline(disp, polygon)

            # Run detectors only on new frames
            if new_frame:
                if red_params is not None:
                    last_red = detect_red_marker(frame.image, red_params, global_mask)
                else:
                    last_red = None
                if rat_params is not None:
                    last_rat = detect_rat_hood(frame.image, rat_params, global_mask)
                else:
                    last_rat = None

            draw_detection(disp, last_red, COLOR_RED, "RED")
            draw_detection(disp, last_rat, COLOR_YELLOW, "RAT")

            # Status panel (top-left)
            y = 25
            mask_state = (f"polygon ({len(polygon)} pts)"
                          if global_mask is not None else "(unset)")
            draw_text(disp, f"mask: {mask_state}", (10, y)); y += 22
            if red_params is None:
                draw_text(disp, "RED: not calibrated  (press 'c')",
                          (10, y), color=COLOR_RED)
            else:
                state = "FOUND" if last_red is not None else "MISS "
                draw_text(disp,
                    f"RED: {state}  range={red_params.color_range}  "
                    f"area=[{red_params.min_area},{red_params.max_area}]",
                    (10, y), color=COLOR_RED)
            y += 22
            if rat_params is None:
                draw_text(disp, "RAT: not calibrated  (press 'r')",
                          (10, y), color=COLOR_YELLOW)
            else:
                state = "FOUND" if last_rat is not None else "MISS "
                draw_text(disp,
                    f"RAT: {state}  V<={rat_params.v_threshold}  "
                    f"area=[{rat_params.min_area},{rat_params.max_area}]",
                    (10, y), color=COLOR_YELLOW)

            # Help panel (bottom-left, multi-line)
            help_lines = [
                "calibrate:  m=polygon  w=save_polygon  c=sample_red  r=sample_rat  p=print",
                "red (transient):    [/] range    ,/. min_area    n/N max_area",
                "rat (transient):    ;/' V_thr    </> min_area    b/B max_area",
                "general:    s=screenshot   q/ESC=quit       (config.py = source of truth)",
            ]
            line_h = 20
            panel_h = line_h * len(help_lines) + 8
            disp_h = disp.shape[0]
            for i, line in enumerate(help_lines):
                org = (10, disp_h - panel_h + line_h * (i + 1))
                draw_text(disp, line, org, color=COLOR_WHITE, scale=0.5)

            cv.imshow(win, disp)
            key = cv.waitKey(1) & 0xFF

            # ----- key handling -----

            if key == 255:
                continue

            if key in (ord("q"), 27):
                break

            elif key == ord("m"):
                f = get_freshest_blocking(grabber)
                if f is not None:
                    polygon, global_mask = calibrate_polygon_mask(f.image)
                    if not polygon:
                        global_mask = None
                        print("[test] arena polygon cleared")
                    else:
                        print(f"[test] arena polygon set ({len(polygon)} vertices)")

            elif key == ord("w"):
                fh, fw = frame.image.shape[:2]
                red_lab = (
                    tuple(red_params.target_lab.astype(int).tolist())
                    if red_params is not None else None
                )
                rat_thr = (
                    int(rat_params.v_threshold)
                    if rat_params is not None else None
                )
                calib = SessionCalibration(
                    frame_size=(fh, fw),
                    arena_polygon=polygon,
                    red_target_lab=red_lab,
                    rat_v_threshold=rat_thr,
                )
                try:
                    save_calibration(args.calib, calib)
                except Exception as e:
                    print(f"[test] save failed: {e}")

            elif key == ord("c"):
                f = get_freshest_blocking(grabber)
                if f is not None:
                    try:
                        target_lab = calibrate_red_marker_color(f.image)
                        red_params = make_red_params(target_lab)
                    except RuntimeError as e:
                        print(f"[test] {e}")

            elif key == ord("r"):
                f = get_freshest_blocking(grabber)
                if f is not None:
                    try:
                        thr = calibrate_rat_threshold(f.image)
                        rat_params = make_rat_params(thr)
                    except RuntimeError as e:
                        print(f"[test] {e}")

            # red color-range  [   ]
            elif key == ord("[") and red_params is not None:
                red_params.color_range = max(5, red_params.color_range - 5)
                print(f"[test] red color_range -> {red_params.color_range}")
            elif key == ord("]") and red_params is not None:
                red_params.color_range = min(120, red_params.color_range + 5)
                print(f"[test] red color_range -> {red_params.color_range}")

            # rat V threshold  ;  '
            elif key == ord(";") and rat_params is not None:
                rat_params.v_threshold = max(5, rat_params.v_threshold - 5)
                print(f"[test] rat v_threshold -> {rat_params.v_threshold}")
            elif key == ord("'") and rat_params is not None:
                rat_params.v_threshold = min(250, rat_params.v_threshold + 5)
                print(f"[test] rat v_threshold -> {rat_params.v_threshold}")

            # red min_area  ,  .
            elif key == ord(",") and red_params is not None:
                red_params.min_area = max(10, red_params.min_area - 50)
                print(f"[test] red min_area -> {red_params.min_area}")
            elif key == ord(".") and red_params is not None:
                red_params.min_area += 50
                print(f"[test] red min_area -> {red_params.min_area}")

            # red max_area  n  N
            elif key == ord("n") and red_params is not None:
                red_params.max_area = max(red_params.min_area + 50,
                                          red_params.max_area - 200)
                print(f"[test] red max_area -> {red_params.max_area}")
            elif key == ord("N") and red_params is not None:
                red_params.max_area += 200
                print(f"[test] red max_area -> {red_params.max_area}")

            # rat min_area  <  >
            elif key == ord("<") and rat_params is not None:
                rat_params.min_area = max(10, rat_params.min_area - 50)
                print(f"[test] rat min_area -> {rat_params.min_area}")
            elif key == ord(">") and rat_params is not None:
                rat_params.min_area += 50
                print(f"[test] rat min_area -> {rat_params.min_area}")

            # rat max_area  b  B
            elif key == ord("b") and rat_params is not None:
                rat_params.max_area = max(rat_params.min_area + 50,
                                          rat_params.max_area - 200)
                print(f"[test] rat max_area -> {rat_params.max_area}")
            elif key == ord("B") and rat_params is not None:
                rat_params.max_area += 200
                print(f"[test] rat max_area -> {rat_params.max_area}")

            elif key == ord("p"):
                print("\n========== current params ==========")
                print(f"polygon vertices = {polygon}")
                if red_params is not None:
                    print(f"red target_lab = {red_params.target_lab.astype(int).tolist()}")
                    print(f"red color_range = {red_params.color_range}")
                    print(f"red min_area    = {red_params.min_area}")
                    print(f"red max_area    = {red_params.max_area}")
                if rat_params is not None:
                    print(f"rat v_threshold = {rat_params.v_threshold}")
                    print(f"rat min_area    = {rat_params.min_area}")
                    print(f"rat max_area    = {rat_params.max_area}")
                print("====================================\n")

            elif key == ord("s"):
                save_idx += 1
                fname = f"test_detection_{save_idx:03d}.png"
                cv.imwrite(fname, disp)
                print(f"[test] saved {fname}")

    finally:
        cv.destroyAllWindows()
        grabber.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
