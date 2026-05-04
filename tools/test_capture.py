"""
tools/test_capture.py
Standalone test for the FrameGrabber.

Run before integrating with detection. This verifies:
  1. Your webcam opens at the requested resolution + framerate
  2. The threaded capture is delivering fresh frames
  3. The display loop can keep up

Usage (from project root):
    python -m tools.test_capture
    python -m tools.test_capture --index 1            # if you have multiple cameras
    python -m tools.test_capture --width 1280 --height 720 --fps 30

Keys (in the live window):
    q or ESC    quit
    s           save current frame to ./test_capture_<idx>.png

The window title shows live capture FPS and the lag between camera-side
frame and the display loop. Lag should stay near 0 — if it grows, the
display can't keep up (rare; usually means a CPU is pegged elsewhere).
"""

import argparse
import sys
import time

import cv2 as cv

# Allow running as `python -m tools.test_capture` from project root
sys.path.insert(0, ".")

import config
from capture import FrameGrabber


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--index", type=int, default=config.CAMERA_INDEX)
    p.add_argument("--width", type=int, default=config.CAMERA_WIDTH)
    p.add_argument("--height", type=int, default=config.CAMERA_HEIGHT)
    p.add_argument("--fps", type=int, default=config.CAMERA_FPS)
    p.add_argument("--fourcc", type=str, default=config.CAMERA_FOURCC)
    args = p.parse_args()

    grabber = FrameGrabber(
        camera_index=args.index,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
    )

    try:
        grabber.open()
    except Exception as e:
        print(f"[error] {e}")
        return 1

    grabber.start()

    win = "test_capture (q to quit, s to save)"
    cv.namedWindow(win, cv.WINDOW_AUTOSIZE)

    last_frame_idx = -1
    last_new_frame_time = time.monotonic()
    last_save_idx = 0
    display_count = 0
    display_t0 = time.monotonic()
    display_fps = 0.0
    STALE_WARN_S = 0.1  # warn only if no new frame for >100 ms

    print("[test] Capture started. Press 'q' or ESC in the window to quit.")

    try:
        while True:
            frame = grabber.get_latest()
            if frame is None:
                # Camera hasn't produced a frame yet
                if cv.waitKey(10) & 0xFF in (ord("q"), 27):
                    break
                continue

            new_frame = (frame.frame_idx != last_frame_idx)
            now = time.monotonic()
            if new_frame:
                last_frame_idx = frame.frame_idx
                last_new_frame_time = now
                display_count += 1

            # Display FPS counter (counts new frames only; same-frame redraws
            # don't count, so this matches camera-side throughput)
            elapsed = now - display_t0
            if elapsed >= 1.0:
                display_fps = display_count / elapsed
                display_count = 0
                display_t0 = now

            # Skip re-rendering an unchanged frame; just poll for keypress.
            # This is also what the real pipeline will do — only process
            # genuinely new frames.
            if not new_frame:
                key = cv.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                continue

            cap_fps = grabber.measured_fps()
            lag_ms = (now - frame.timestamp) * 1000.0
            since_new_ms = (now - last_new_frame_time) * 1000.0

            disp = frame.image.copy()
            txt = (f"frame {frame.frame_idx} | cap {cap_fps:5.1f} fps | "
                   f"disp {display_fps:5.1f} fps | lag {lag_ms:5.1f} ms")
            cv.putText(disp, txt, (10, 25),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv.LINE_AA)
            cv.putText(disp, txt, (10, 25),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv.LINE_AA)
            # Only flag STALE if frames have been missing for an unusually
            # long time — at 30 fps a normal gap is ~33 ms, so >100 ms
            # means real frame drop, not just display-faster-than-camera.
            if since_new_ms > STALE_WARN_S * 1000:
                cv.putText(disp, f"NO NEW FRAME ({since_new_ms:.0f} ms)",
                           (10, 55), cv.FONT_HERSHEY_SIMPLEX, 0.6,
                           (0, 0, 255), 2, cv.LINE_AA)

            cv.imshow(win, disp)
            key = cv.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                last_save_idx += 1
                fname = f"test_capture_{last_save_idx:03d}.png"
                cv.imwrite(fname, frame.image)
                print(f"[test] saved {fname}")
    finally:
        cv.destroyAllWindows()
        grabber.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
