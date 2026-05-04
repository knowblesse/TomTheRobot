"""
tools/test_tracker.py
Standalone tracker test against synthetic motion.

Two simulated targets move in pre-defined paths with added measurement
noise. The tracker is fed those noisy measurements and we visualize:

  - True path (faint background trail)
  - Noisy detections (small red/yellow dots)
  - Kalman-filtered estimates (large red/yellow markers)
  - Predicted positions during simulated occlusion / dropouts

This validates the tracker without needing the camera or the robot.
After this looks right on synthetic data, we'll integrate it with real
detections from the live camera.

Keys:
    q / ESC   quit
    space     pause / resume
    o         force occlusion for ~30 frames (both targets close, one detection)
    d         force a 20-frame detection dropout (no measurements)
    r         reset simulation
"""

import argparse
import sys

import cv2 as cv
import numpy as np

sys.path.insert(0, ".")

import config
from detection import Detection
from tracker import TrackerConfig, TwoObjectTracker


# ----- synthetic world -----

W, H = 800, 500
RAT_NOISE_PX = 2.0       # detection jitter for rat
ROBOT_NOISE_PX = 1.0
DT = 1.0 / 30.0


def rat_path(t: float):
    """Erratic rat: figure-eight with random twitches."""
    x = 400 + 250 * np.sin(0.6 * t)
    y = 250 + 80 * np.sin(1.2 * t)
    # add jerks every couple seconds
    x += 30 * np.sin(8 * t) * np.exp(-((t % 2.0) - 1.0) ** 2 * 4)
    return x, y


def robot_path(t: float):
    """Predictable robot: slow circle."""
    x = 400 + 200 * np.cos(0.5 * t)
    y = 250 + 130 * np.sin(0.5 * t)
    return x, y


def fake_detection(x: float, y: float, noise: float) -> Detection:
    """Create a Detection-like object with a tiny placeholder contour."""
    nx = x + np.random.randn() * noise
    ny = y + np.random.randn() * noise
    cnt = np.array([[[int(nx) - 5, int(ny) - 5]],
                    [[int(nx) + 5, int(ny) - 5]],
                    [[int(nx) + 5, int(ny) + 5]],
                    [[int(nx) - 5, int(ny) + 5]]], dtype=np.int32)
    return Detection(x=nx, y=ny, area=100.0, contour=cnt)


# ----- visualization -----

def color_for_status(base_bgr, status):
    """Dim color when predicted, gray when lost."""
    if status == "detected":
        return base_bgr
    if status == "predicted":
        return tuple(int(c * 0.6) for c in base_bgr)
    return (120, 120, 120)


def draw_text(img, text, org, color=(0, 255, 255), scale=0.5):
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               (0, 0, 0), 3, cv.LINE_AA)
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               color, 1, cv.LINE_AA)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()
    np.random.seed(args.seed)

    cfg = TrackerConfig(
        process_noise_rat=config.KALMAN_PROCESS_NOISE_RAT,
        process_noise_robot=config.KALMAN_PROCESS_NOISE_ROBOT,
        measurement_noise=config.KALMAN_MEASUREMENT_NOISE,
        max_frames_lost=config.MAX_FRAMES_LOST,
        association_gate_px=config.ASSOCIATION_GATE_PX,
        occlusion_distance_px=config.OCCLUSION_DISTANCE_PX,
    )
    tracker = TwoObjectTracker(cfg)

    win = "test_tracker (synthetic)"
    cv.namedWindow(win, cv.WINDOW_AUTOSIZE)
    canvas = np.full((H, W, 3), 30, dtype=np.uint8)

    paused = False
    occlusion_frames = 0   # countdown
    dropout_frames = 0     # countdown
    sim_t = 0.0

    rat_trail = []
    robot_trail = []
    rat_filt_trail = []
    robot_filt_trail = []

    while True:
        if not paused:
            sim_t += DT

            # Ground truth
            tx_r, ty_r = rat_path(sim_t)
            tx_b, ty_b = robot_path(sim_t)

            # Force occlusion: drag robot toward rat for occlusion_frames
            if occlusion_frames > 0:
                tx_b = 0.6 * tx_b + 0.4 * tx_r
                ty_b = 0.6 * ty_b + 0.4 * ty_r
                occlusion_frames -= 1
                # Provide only one detection during forced occlusion
                rat_det = fake_detection(tx_r, ty_r, RAT_NOISE_PX)
                robot_det = None
            elif dropout_frames > 0:
                rat_det = None
                robot_det = None
                dropout_frames -= 1
            else:
                rat_det = fake_detection(tx_r, ty_r, RAT_NOISE_PX)
                robot_det = fake_detection(tx_b, ty_b, ROBOT_NOISE_PX)

            rat_state, robot_state, occluded = tracker.update(rat_det, robot_det, DT)

            rat_trail.append((tx_r, ty_r))
            robot_trail.append((tx_b, ty_b))
            rat_filt_trail.append((rat_state.x, rat_state.y))
            robot_filt_trail.append((robot_state.x, robot_state.y))
            for tr in (rat_trail, robot_trail, rat_filt_trail, robot_filt_trail):
                if len(tr) > 200:
                    del tr[0]

        # Render
        disp = canvas.copy()

        # Trails: ground truth (faint), filtered (bright)
        def draw_trail(trail, color, thickness=1):
            if len(trail) < 2:
                return
            pts = np.array(trail, dtype=np.int32).reshape(-1, 1, 2)
            cv.polylines(disp, [pts], False, color, thickness, cv.LINE_AA)

        draw_trail(rat_trail, (40, 40, 40), 1)
        draw_trail(robot_trail, (40, 40, 40), 1)
        draw_trail(rat_filt_trail, (0, 200, 200), 1)
        draw_trail(robot_filt_trail, (0, 0, 200), 1)

        # Current detections (small dots)
        if not paused:
            if rat_det is not None:
                cv.circle(disp, (int(rat_det.x), int(rat_det.y)), 3,
                          (0, 255, 255), 1)
            if robot_det is not None:
                cv.circle(disp, (int(robot_det.x), int(robot_det.y)), 3,
                          (0, 0, 255), 1)

        # Filtered estimates (cross markers)
        rc = color_for_status((0, 255, 255), rat_state.status)
        bc = color_for_status((0, 0, 255), robot_state.status)
        cv.drawMarker(disp, (int(rat_state.x), int(rat_state.y)), rc,
                      markerType=cv.MARKER_CROSS, markerSize=22, thickness=2)
        cv.drawMarker(disp, (int(robot_state.x), int(robot_state.y)), bc,
                      markerType=cv.MARKER_CROSS, markerSize=22, thickness=2)

        # Status panel (top)
        draw_text(disp,
                  f"RAT  {rat_state.status:>9s}  miss={rat_state.frames_since_detection:>3d}  "
                  f"v=({rat_state.vx:+5.0f},{rat_state.vy:+5.0f}) px/s",
                  (10, 25), color=(0, 255, 255))
        draw_text(disp,
                  f"BOT  {robot_state.status:>9s}  miss={robot_state.frames_since_detection:>3d}  "
                  f"v=({robot_state.vx:+5.0f},{robot_state.vy:+5.0f}) px/s",
                  (10, 50), color=(0, 0, 255))
        if occluded:
            draw_text(disp, "OCCLUSION", (10, 75), color=(0, 200, 0))
        if dropout_frames > 0:
            draw_text(disp, f"DROPOUT ({dropout_frames})", (10, 95),
                      color=(0, 200, 0))

        # Help panel (bottom)
        help_lines = [
            "q/ESC=quit   space=pause   o=force occlusion   d=force dropout   r=reset",
            "yellow = rat / red = robot   small dots = noisy detections   crosses = filtered",
        ]
        for i, line in enumerate(help_lines):
            org = (10, H - 12 - 18 * (len(help_lines) - 1 - i))
            draw_text(disp, line, org, color=(255, 255, 255), scale=0.45)

        cv.imshow(win, disp)
        key = cv.waitKey(int(DT * 1000)) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord(" "):
            paused = not paused
        elif key == ord("o"):
            occlusion_frames = 30
        elif key == ord("d"):
            dropout_frames = 20
        elif key == ord("r"):
            tracker = TwoObjectTracker(cfg)
            sim_t = 0.0
            rat_trail.clear(); robot_trail.clear()
            rat_filt_trail.clear(); robot_filt_trail.clear()

    cv.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
