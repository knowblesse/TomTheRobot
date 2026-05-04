"""
main.py
End-to-end closed-loop rat tracker with RVR+ chase control.

Threads:
  - main thread:           UI (cv2 windows + key handling)
  - capture thread:        FrameGrabber (always-freshest frame)
  - processing thread:     detection + tracker + controller decisions
  - control thread:        RVR+ Bluetooth I/O

The processing thread writes commands to a small queue; the control
thread drains the queue and forwards to the robot. This decouples
vision timing from BLE timing — a slow BLE round-trip never blocks
the camera/tracker.

Pre-flight: calibration is done before threads start.
  1. Auto-load arena polygon from last_calibration.json (or press 'm')
  2. Sample red marker color (press 'c')
  3. Sample rat threshold (press 'r')
  4. Connect to RVR+ and reset_aim
  5. Start threads, enter main loop

Keys (in main GUI):
  q / ESC    quit cleanly
  space      emergency stop (force IDLE; press toggle to resume)
  r-key      toggle chase on/off  (config: KEY_TOGGLE_FOLLOW = 'r')
  d          toggle live display detail
  m          re-draw arena polygon
  c          re-sample red marker color
  s          screenshot (with overlays) to ./session_<idx>.png
"""

import argparse
import math
import queue
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2 as cv
import numpy as np

import config
from arduino_io import ArduinoIO, NoOpArduinoIO, connect as connect_arduino
from calibration_io import SessionCalibration, load_calibration, save_calibration
from capture import FrameGrabber
from comms import RobotInterface, MockRobot
from controller import ChaseController, StopReason
from detection import (
    Detection,
    RatHoodParams,
    RedMarkerParams,
    calibrate_polygon_mask,
    calibrate_rat_threshold,
    calibrate_red_marker_color,
    detect_rat_hood,
    detect_red_marker,
    polygon_to_mask,
)
from tracker import TrackerConfig, TwoObjectTracker
from world_state import Command, WorldState

DEFAULT_CALIB_PATH = "./last_calibration.json"


# -----------------------------------------------------------------
#                       Helpers
# -----------------------------------------------------------------

COLOR_RED = (0, 0, 255)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_GREEN = (0, 200, 0)
COLOR_BLUE = (255, 100, 0)


def draw_text(img, text, org, color=COLOR_YELLOW, scale=0.55):
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               COLOR_BLACK, 3, cv.LINE_AA)
    cv.putText(img, text, org, cv.FONT_HERSHEY_SIMPLEX, scale,
               color, 1, cv.LINE_AA)


def get_freshest_blocking(grabber: FrameGrabber, timeout_s: float = 2.0):
    t0 = time.monotonic()
    last_idx = -1
    while time.monotonic() - t0 < timeout_s:
        f = grabber.get_latest()
        if f is not None and f.frame_idx != last_idx:
            return f
        time.sleep(0.01)
    return grabber.get_latest()


def _direction_label(offset_deg: float) -> str:
    """Render a camera-frame angle in plain English for user prompts.
    0=up (top of image), 90=right, 180=down, 270=left. Snaps to the
    cardinal name if within a few degrees, otherwise gives the angle."""
    a = float(offset_deg) % 360.0
    cardinals = [
        (0,   "the TOP of the image (image-up)"),
        (90,  "the RIGHT of the image (image-right)"),
        (180, "the BOTTOM of the image (image-down)"),
        (270, "the LEFT of the image (image-left)"),
    ]
    for ref, label in cardinals:
        if min(abs(a - ref), 360 - abs(a - ref)) < 5.0:
            return label
    return f"camera-frame angle {a:.0f}\u00b0"


# -----------------------------------------------------------------
#                       Processing thread
# -----------------------------------------------------------------

def processing_loop(
    grabber: FrameGrabber,
    state: WorldState,
    cmd_queue: "queue.Queue[Command]",
    red_params_box: list,
    rat_params_box: list,
    global_mask_box: list,
    tracker: TwoObjectTracker,
    controller: ChaseController,
    last_stop_reason_box: list,
):
    """Run detection + tracker + controller for every fresh frame."""
    last_idx = -1
    last_t = None
    while state.running:
        frame = grabber.get_latest()
        if frame is None or frame.frame_idx == last_idx:
            time.sleep(0.001)
            continue
        last_idx = frame.frame_idx

        now = frame.timestamp
        dt = (now - last_t) if last_t is not None else (1.0 / config.CAMERA_FPS)
        last_t = now

        red_params = red_params_box[0]
        rat_params = rat_params_box[0]
        global_mask = global_mask_box[0]

        rat_det: Optional[Detection] = None
        robot_det: Optional[Detection] = None
        if red_params is not None:
            robot_det = detect_red_marker(frame.image, red_params, global_mask)
        if rat_params is not None:
            rat_det = detect_rat_hood(frame.image, rat_params, global_mask)

        rat_state, robot_state, occluded = tracker.update(rat_det, robot_det, dt)

        # Controller decision
        cmd, stop_reason = controller.decide(rat_state, robot_state, now)

        # Publish to shared state
        rat_raw = (rat_det.x, rat_det.y) if rat_det is not None else (None, None)
        robot_raw = (robot_det.x, robot_det.y) if robot_det is not None else (None, None)
        with state.lock:
            state.timestamp = now
            state.frame_idx = frame.frame_idx
            state.rat = rat_state
            state.robot = robot_state
            state.rat_raw_xy = rat_raw
            state.robot_raw_xy = robot_raw
            # Persist last-observed area; only overwrite when a fresh
            # detection arrives, so we see the size right up until detection
            # was lost (useful for area-threshold tuning).
            if rat_det is not None:
                state.rat_last_area = float(rat_det.area)
            if robot_det is not None:
                state.robot_last_area = float(robot_det.area)
            state.occlusion = occluded
            state.dropped_frame = False
            if cmd is not None:
                state.last_command = cmd

        if cmd is not None:
            try:
                cmd_queue.put_nowait(cmd)
            except queue.Full:
                pass  # drop — heartbeat will resend
        if stop_reason is not None:
            last_stop_reason_box[0] = (stop_reason, now)


# -----------------------------------------------------------------
#                       Control thread
# -----------------------------------------------------------------

def control_loop(
    state: WorldState,
    robot: RobotInterface,
    cmd_queue: "queue.Queue[Command]",
    ready_evt: threading.Event,
    reset_aim_evt: threading.Event,
    reset_aim_request_evt: threading.Event,
    connect_error_box: list,
):
    """Owns the robot lifecycle. We connect HERE, on this thread, rather
    than on the main thread, because cv2's DirectShow backend initializes
    the main thread in COM single-threaded-apartment (STA) mode, which is
    incompatible with bleak on Windows. Doing BLE on a fresh thread lets
    bleak configure it as MTA correctly.

    Sequence:
      1. connect()
      2. set ready_evt; main thread now knows it can proceed
      3. wait for reset_aim_evt (main thread sets after user confirms)
      4. reset_aim()
      5. enter command-drain loop
      6. on shutdown: stop() + disconnect()

    Mid-session: if reset_aim_request_evt fires, do reset_aim() between
    queue waits. Main thread is responsible for stopping the robot
    BEFORE it sets the request, so the IMU baseline is recorded while
    stationary.
    """
    try:
        robot.connect()
    except Exception as e:
        connect_error_box[0] = e
        ready_evt.set()  # unblock main so it can see the error
        return

    ready_evt.set()

    # Wait for main thread to prompt the user and request reset_aim.
    # Bail early if main is already shutting down.
    while not reset_aim_evt.is_set():
        if not state.running:
            try:
                robot.disconnect()
            except Exception:
                pass
            return
        time.sleep(0.05)

    try:
        robot.reset_aim()
    except Exception as e:
        print(f"[control] reset_aim failed: {e}")

    last_sent_heading: Optional[int] = None
    last_sent_speed: Optional[int] = None
    last_was_stop = True

    while state.running:
        # Check for a mid-session re-aim request between command ticks
        if reset_aim_request_evt.is_set():
            reset_aim_request_evt.clear()
            try:
                robot.reset_aim()
                print("[control] reset_aim done (mid-session)")
            except Exception as e:
                print(f"[control] mid-session reset_aim failed: {e}")

        try:
            cmd = cmd_queue.get(timeout=0.1)
        except queue.Empty:
            continue

        if not state.robot_enabled:
            # Manual stop active: silently swallow drive commands but
            # always honor stop.
            if cmd.stop and not last_was_stop:
                try:
                    robot.stop()
                except Exception as e:
                    print(f"[control] stop failed: {e}")
                last_was_stop = True
                last_sent_heading = None
                last_sent_speed = None
            continue

        try:
            if cmd.stop:
                if not last_was_stop:
                    robot.stop()
                last_was_stop = True
                # Invalidate cached values so the next non-stop command
                # re-sends both heading and speed unconditionally.
                # Without this, the robot rotates but never resumes
                # speed when a new chase starts at the same speed value.
                last_sent_heading = None
                last_sent_speed = None
            else:
                # Update heading and speed; only send if they actually changed
                if cmd.heading != last_sent_heading:
                    robot.set_heading(cmd.heading)
                    last_sent_heading = cmd.heading
                if cmd.speed != last_sent_speed:
                    robot.set_speed(cmd.speed)
                    last_sent_speed = cmd.speed
                last_was_stop = False
        except Exception as e:
            print(f"[control] command failed: {e}")

    # Shutdown: stop + disconnect on this thread (so disconnect() also
    # runs on the MTA-friendly thread).
    try:
        robot.stop()
    except Exception:
        pass
    try:
        robot.disconnect()
    except Exception:
        pass


# -----------------------------------------------------------------
#                       Display
# -----------------------------------------------------------------

def render(
    frame_bgr: np.ndarray,
    state: WorldState,
    polygon: list,
    global_mask: Optional[np.ndarray],
    controller: ChaseController,
    last_stop_reason_box: list,
    grabber_fps: float,
    red_params_box: list,
    rat_params_box: list,
) -> np.ndarray:
    """Compose the display image with overlays."""
    disp = frame_bgr.copy()

    # Dim outside polygon
    if global_mask is not None:
        inv = cv.bitwise_not(global_mask)
        outside = cv.bitwise_and(disp, disp, mask=inv)
        outside = (outside * 0.3).astype(np.uint8)
        inside = cv.bitwise_and(disp, disp, mask=global_mask)
        disp = cv.add(inside, outside)
        if len(polygon) >= 2:
            pts = np.array(polygon, dtype=np.int32).reshape(-1, 1, 2)
            cv.polylines(disp, [pts], isClosed=True, color=(0, 200, 200),
                         thickness=1, lineType=cv.LINE_AA)

    snap = state.snapshot()

    # ---- Raw detection markers (instant, no Kalman smoothing) ----
    # These show what the detector found *this frame*. If they move
    # smoothly with the rat/robot, detection is fast. If the filtered
    # marker (below) lags behind these, that's Kalman smoothing.
    rrx, rry = snap.rat_raw_xy
    if rrx is not None:
        cv.circle(disp, (int(rrx), int(rry)), 4, COLOR_YELLOW, 1, cv.LINE_AA)
    brx, bry = snap.robot_raw_xy
    if brx is not None:
        cv.circle(disp, (int(brx), int(bry)), 4, COLOR_RED, 1, cv.LINE_AA)

    # ---- Filtered (Kalman) markers — what the controller uses ----
    # Draw rat
    rc_base = COLOR_YELLOW
    if snap.rat.status == "predicted":
        rc = (0, 180, 180)
    elif snap.rat.status == "lost":
        rc = (100, 100, 100)
    else:
        rc = rc_base
    cv.drawMarker(disp, (int(snap.rat.x), int(snap.rat.y)), rc,
                  markerType=cv.MARKER_CROSS, markerSize=18, thickness=2)

    # Draw robot
    bc_base = COLOR_RED
    if snap.robot.status == "predicted":
        bc = (0, 0, 180)
    elif snap.robot.status == "lost":
        bc = (100, 100, 100)
    else:
        bc = bc_base
    cv.drawMarker(disp, (int(snap.robot.x), int(snap.robot.y)), bc,
                  markerType=cv.MARKER_DIAMOND, markerSize=18, thickness=2)

    # Line from robot to rat (chase target)
    if snap.rat.status != "lost" and snap.robot.status != "lost":
        cv.arrowedLine(disp,
                       (int(snap.robot.x), int(snap.robot.y)),
                       (int(snap.rat.x), int(snap.rat.y)),
                       (0, 255, 0) if controller.mode == "chasing" else (80, 80, 80),
                       1, cv.LINE_AA, tipLength=0.05)

    # Status panel
    y = 22
    mode_color = COLOR_GREEN if controller.mode == "chasing" else (180, 180, 180)
    if controller.mode == "chasing":
        sub = "near" if controller.near else "driving"
        mode_text = f"MODE: CHASING ({sub})"
    else:
        mode_text = "MODE: IDLE"
    draw_text(disp, mode_text, (10, y), color=mode_color); y += 22

    rat_p = rat_params_box[0]
    red_p = red_params_box[0]

    rat_area_str = (f"size={int(snap.rat_last_area)} "
                    f"[{rat_p.min_area},{rat_p.max_area}]"
                    if rat_p is not None else "")
    draw_text(disp,
              f"RAT  {snap.rat.status}  {rat_area_str}",
              (10, y), color=COLOR_YELLOW); y += 22

    bot_area_str = (f"size={int(snap.robot_last_area)} "
                    f"[{red_p.min_area},{red_p.max_area}]"
                    if red_p is not None else "")
    draw_text(disp,
              f"BOT  {snap.robot.status}  {bot_area_str}",
              (10, y), color=COLOR_RED); y += 22

    if snap.rat.status != "lost" and snap.robot.status != "lost":
        d = math.hypot(snap.rat.x - snap.robot.x, snap.rat.y - snap.robot.y)
        draw_text(disp, f"distance: {d:.0f} px  (stop< {config.STOP_DISTANCE_PX})",
                  (10, y), color=COLOR_WHITE); y += 22
    last_cmd = snap.last_command
    if last_cmd.stop:
        draw_text(disp, "last cmd: STOP", (10, y), color=COLOR_WHITE); y += 22
    else:
        draw_text(disp, f"last cmd: speed={last_cmd.speed} heading={last_cmd.heading}",
                  (10, y), color=COLOR_WHITE); y += 22
    draw_text(disp, f"FPS: {grabber_fps:.1f}", (10, y), color=COLOR_WHITE); y += 22
    if not snap.robot_enabled:
        draw_text(disp, "ROBOT DISABLED (press SPACE to enable)",
                  (10, y), color=(0, 100, 255)); y += 22
    if snap.occlusion:
        draw_text(disp, "OCCLUSION", (10, y), color=COLOR_GREEN); y += 22

    # Stop reason banner (sticks for 3 seconds after stop event)
    sr_entry = last_stop_reason_box[0]
    if sr_entry is not None:
        sr, t = sr_entry
        if time.monotonic() - t < 3.0:
            draw_text(disp, f"STOPPED: {sr.reason}  {sr.detail}",
                      (10, y), color=(0, 100, 255))

    # Help panel (bottom)
    help_lines = [
        "calibrate:  m=polygon  c=red  r=rat  h=reset_aim  w=save",
        "control:    t=toggle_chase   space=robot_disable/enable   q/ESC=quit",
        "red tune:   [/] range    ,/. min_area    n/N max_area",
        "rat tune:   ;/' V_thr    </> min_area    b/B max_area",
        "general:    s=screenshot",
    ]
    line_h = 20
    panel_h = line_h * len(help_lines) + 8
    disp_h = disp.shape[0]
    for i, line in enumerate(help_lines):
        org = (10, disp_h - panel_h + line_h * (i + 1))
        draw_text(disp, line, org, color=COLOR_WHITE, scale=0.5)

    return disp


# -----------------------------------------------------------------
#                       Pre-flight calibration
# -----------------------------------------------------------------

def preflight(
    grabber: FrameGrabber,
    calib_path: str,
):
    """Interactive pre-flight: get polygon, red LAB color, rat V threshold.

    Auto-loads any of these that are present in last_calibration.json,
    only prompting for the pieces that are missing.

    Returns (polygon, global_mask, red_params, rat_params, session).
    `session` is the SessionCalibration that should be saved on `w` —
    holds the measured values that go to disk.
    """
    polygon: List[Tuple[int, int]] = []
    global_mask = None
    red_lab: Optional[np.ndarray] = None
    rat_thr: Optional[int] = None

    try:
        loaded = load_calibration(calib_path)
    except ValueError as e:
        print(f"[main] {e}")
        loaded = None

    if loaded is not None:
        polygon = list(loaded.arena_polygon)
        if polygon:
            global_mask = polygon_to_mask(polygon, loaded.frame_size)
        if loaded.red_target_lab is not None:
            red_lab = np.array(loaded.red_target_lab, dtype=np.float32)
        if loaded.rat_v_threshold is not None:
            rat_thr = int(loaded.rat_v_threshold)

    # Prompt only for missing pieces.
    if global_mask is None:
        print("[main] no polygon saved; please draw arena.")
        f = get_freshest_blocking(grabber)
        polygon, global_mask = calibrate_polygon_mask(f.image)

    if red_lab is None:
        print("[main] no red sample saved; please sample.")
        while red_lab is None:
            f = get_freshest_blocking(grabber)
            red_lab = calibrate_red_marker_color(f.image)
            if red_lab is None:
                print("[main] red sample is required; please try again.")
    else:
        print(f"[main] red sample restored: LAB={red_lab.astype(int).tolist()}")

    if rat_thr is None:
        print("[main] no rat threshold saved; please sample.")
        while rat_thr is None:
            f = get_freshest_blocking(grabber)
            rat_thr = calibrate_rat_threshold(f.image)
            if rat_thr is None:
                print("[main] rat threshold is required; please try again.")
    else:
        print(f"[main] rat threshold restored: V<={rat_thr}")

    red_params = RedMarkerParams(
        target_lab=red_lab,
        color_range=config.ROBOT_COLOR_RANGE,
        min_area=config.ROBOT_MIN_AREA_PX,
        max_area=config.ROBOT_MAX_AREA_PX,
    )
    rat_params = RatHoodParams(
        v_threshold=rat_thr,
        min_area=config.RAT_MIN_AREA_PX,
        max_area=config.RAT_MAX_AREA_PX,
    )

    # Build the session object that will be saved on 'w'. We persist
    # the *measured* values; numeric tuning lives in config.py.
    h_now, w_now = (grabber.get_latest().image.shape[:2]
                    if grabber.get_latest() is not None
                    else (config.CAMERA_HEIGHT, config.CAMERA_WIDTH))
    session = SessionCalibration(
        frame_size=(h_now, w_now),
        arena_polygon=polygon,
        red_target_lab=tuple(red_lab.astype(int).tolist()),
        rat_v_threshold=rat_thr,
    )

    # Save right now so the file always reflects what we just used.
    save_calibration(calib_path, session)

    return polygon, global_mask, red_params, rat_params, session


# -----------------------------------------------------------------
#                       Main
# -----------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mock", action="store_true", help="MockRobot (no hardware)")
    p.add_argument("--index", type=int, default=config.CAMERA_INDEX)
    p.add_argument("--calib", type=str, default=DEFAULT_CALIB_PATH)
    p.add_argument("--arduino-port", type=str, default=config.ARDUINO_PORT,
                   help="Override Arduino COM port (else config or auto-detect)")
    p.add_argument("--no-arduino", action="store_true",
                   help="Skip Arduino entirely (digital I/O becomes no-op)")
    args = p.parse_args()

    # 0. Connect to Arduino first (very beginning per spec). This is a
    # soft requirement: if the board is missing or unresponsive, we
    # continue with a NoOpArduinoIO so the rest of the rig still runs
    # and the user gets a console warning instead of a crash.
    if args.no_arduino:
        print("[main] --no-arduino: skipping serial I/O setup")
        arduino: ArduinoIO | NoOpArduinoIO = NoOpArduinoIO()
    else:
        arduino_real = connect_arduino(
            port=args.arduino_port,
            baud=config.ARDUINO_BAUD,
            boot_wait_s=config.ARDUINO_BOOT_WAIT_S,
            ping_timeout_s=config.ARDUINO_PING_TIMEOUT_S,
        )
        if arduino_real is None:
            print("[main] WARNING: Arduino not connected; digital I/O disabled")
            arduino = NoOpArduinoIO()
        else:
            arduino = arduino_real

    # 1. Open camera
    grabber = FrameGrabber(
        camera_index=args.index,
        width=config.CAMERA_WIDTH,
        height=config.CAMERA_HEIGHT,
        fps=config.CAMERA_FPS,
        fourcc=config.CAMERA_FOURCC,
    )
    grabber.open()
    grabber.start()

    # 2. Pre-flight calibration
    polygon, global_mask, red_params, rat_params, session = preflight(
        grabber, args.calib
    )

    # 3. Build robot interface (do NOT connect on main thread — bleak on
    # Windows requires MTA threading, which cv2's DirectShow backend
    # has already broken on the main thread). Connection happens inside
    # the control thread.
    if args.mock:
        robot: RobotInterface = MockRobot(verbose=True)
    else:
        from comms.rvr_bluetooth import RvrBluetooth
        robot = RvrBluetooth(timeout_s=10.0)

    # 4. Build tracker + controller
    tracker_cfg = TrackerConfig(
        process_noise_rat=config.KALMAN_PROCESS_NOISE_RAT,
        process_noise_robot=config.KALMAN_PROCESS_NOISE_ROBOT,
        measurement_noise=config.KALMAN_MEASUREMENT_NOISE,
        max_frames_lost=config.MAX_FRAMES_LOST,
        association_gate_px=config.ASSOCIATION_GATE_PX,
        occlusion_distance_px=config.OCCLUSION_DISTANCE_PX,
    )
    tracker = TwoObjectTracker(tracker_cfg)
    controller = ChaseController(imu_offset_deg=config.INITIAL_IMU_OFFSET_DEG)

    # 5. Shared state + threads
    state = WorldState()
    cmd_queue: queue.Queue = queue.Queue(maxsize=4)
    red_params_box = [red_params]
    rat_params_box = [rat_params]
    global_mask_box = [global_mask]
    last_stop_reason_box: list = [None]

    ready_evt = threading.Event()
    reset_aim_evt = threading.Event()
    reset_aim_request_evt = threading.Event()
    connect_error_box: list = [None]

    ctrl_thread = threading.Thread(
        target=control_loop,
        args=(state, robot, cmd_queue, ready_evt, reset_aim_evt,
              reset_aim_request_evt, connect_error_box),
        daemon=True, name="control",
    )
    ctrl_thread.start()

    # Wait for the control thread to finish connecting (or error out)
    print("[main] connecting to robot ...")
    ready_evt.wait()
    if connect_error_box[0] is not None:
        print(f"[main] robot connect failed: {connect_error_box[0]}")
        with state.lock:
            state.running = False
        ctrl_thread.join(timeout=2.0)
        grabber.stop()
        return 1

    direction_label = _direction_label(config.INITIAL_IMU_OFFSET_DEG)
    print(f"[main] place robot facing {direction_label}, then press ENTER")
    input()
    reset_aim_evt.set()
    print("[main] robot aim reset; assume IMU=camera frame")

    # Now start the processing thread
    proc_thread = threading.Thread(
        target=processing_loop,
        args=(grabber, state, cmd_queue, red_params_box, rat_params_box,
              global_mask_box, tracker, controller, last_stop_reason_box),
        daemon=True, name="processing",
    )
    proc_thread.start()

    # 6. Main loop (UI)
    win = "rattracker"
    cv.namedWindow(win, cv.WINDOW_AUTOSIZE)
    save_idx = 0
    last_idx = -1

    try:
        while state.running:
            frame = grabber.get_latest()
            if frame is None:
                if cv.waitKey(10) & 0xFF in (config.KEY_QUIT, 27):
                    break
                continue

            disp = render(frame.image, state, polygon, global_mask_box[0],
                          controller, last_stop_reason_box,
                          grabber.measured_fps(),
                          red_params_box, rat_params_box)
            cv.imshow(win, disp)
            key = cv.waitKey(1) & 0xFF

            if key == 255:
                continue
            if key in (config.KEY_QUIT, 27):
                break
            elif key == config.KEY_TOGGLE_FOLLOW:
                # 't' only — 'r' is reserved for re-sample below
                sr = controller.toggle_chase()
                if sr is None:
                    # Just entered CHASING. Fire chase-start pulse.
                    arduino.pulse(config.ARDUINO_PIN_CHASE_START)
                else:
                    last_stop_reason_box[0] = (sr, time.monotonic())
                    # Force a stop command into the queue so the robot
                    # actually halts on the next control tick.
                    try:
                        cmd_queue.put_nowait(Command(speed=0, heading=0, stop=True))
                    except queue.Full:
                        pass
            elif key == config.KEY_STOP_ROBOT:
                with state.lock:
                    state.robot_enabled = not state.robot_enabled
                    enabled = state.robot_enabled
                if not enabled:
                    controller.force_stop()
                    try:
                        cmd_queue.put_nowait(Command(speed=0, heading=0, stop=True))
                    except queue.Full:
                        pass
                print(f"[main] robot_enabled = {enabled}")
            elif key == ord('m'):
                f = get_freshest_blocking(grabber)
                polygon, gm = calibrate_polygon_mask(f.image)
                global_mask_box[0] = gm if polygon else None
                session.arena_polygon = polygon
                session.frame_size = f.image.shape[:2]
            elif key == ord('w'):
                # Persist the current session (polygon + samples) to JSON.
                try:
                    save_calibration(args.calib, session)
                except Exception as e:
                    print(f"[main] save failed: {e}")
            elif key == ord('h'):
                # Mid-session reset_aim. Stop motion first so the IMU
                # records a stationary baseline.
                controller.force_stop()
                try:
                    cmd_queue.put_nowait(Command(speed=0, heading=0, stop=True))
                except queue.Full:
                    pass
                # Brief pause so the stop command actually flushes
                time.sleep(0.3)
                print(f"\n[main] place robot facing "
                      f"{_direction_label(controller.imu_offset_deg)}, "
                      f"then press ENTER")
                try:
                    input()
                except EOFError:
                    pass
                reset_aim_request_evt.set()
                # Show a transient status message
                last_stop_reason_box[0] = (
                    StopReason("reaim", "heading reset"),
                    time.monotonic(),
                )
            elif key == ord('c'):
                f = get_freshest_blocking(grabber)
                tlab = calibrate_red_marker_color(f.image)
                if tlab is None:
                    print("[main] keeping previous red sample")
                else:
                    red_params_box[0] = RedMarkerParams(
                        target_lab=tlab,
                        color_range=config.ROBOT_COLOR_RANGE,
                        min_area=config.ROBOT_MIN_AREA_PX,
                        max_area=config.ROBOT_MAX_AREA_PX,
                    )
                    session.red_target_lab = tuple(tlab.astype(int).tolist())
            elif key == ord('r'):
                f = get_freshest_blocking(grabber)
                thr = calibrate_rat_threshold(f.image)
                if thr is None:
                    print("[main] keeping previous rat threshold")
                else:
                    rat_params_box[0] = RatHoodParams(
                        v_threshold=thr,
                        min_area=config.RAT_MIN_AREA_PX,
                        max_area=config.RAT_MAX_AREA_PX,
                    )
                    session.rat_v_threshold = thr

            # ---- Transient tuning keys (changes don't persist; tell Claude
            # to fold values you like into config.py) ----
            elif key == ord('[') and red_params_box[0] is not None:
                red_params_box[0].color_range = max(5, red_params_box[0].color_range - 5)
                print(f"[main] red color_range -> {red_params_box[0].color_range}")
            elif key == ord(']') and red_params_box[0] is not None:
                red_params_box[0].color_range = min(120, red_params_box[0].color_range + 5)
                print(f"[main] red color_range -> {red_params_box[0].color_range}")
            elif key == ord(',') and red_params_box[0] is not None:
                red_params_box[0].min_area = max(10, red_params_box[0].min_area - 50)
                print(f"[main] red min_area -> {red_params_box[0].min_area}")
            elif key == ord('.') and red_params_box[0] is not None:
                red_params_box[0].min_area += 50
                print(f"[main] red min_area -> {red_params_box[0].min_area}")
            elif key == ord('n') and red_params_box[0] is not None:
                red_params_box[0].max_area = max(red_params_box[0].min_area + 50,
                                                 red_params_box[0].max_area - 200)
                print(f"[main] red max_area -> {red_params_box[0].max_area}")
            elif key == ord('N') and red_params_box[0] is not None:
                red_params_box[0].max_area += 200
                print(f"[main] red max_area -> {red_params_box[0].max_area}")

            elif key == ord(';') and rat_params_box[0] is not None:
                rat_params_box[0].v_threshold = max(5, rat_params_box[0].v_threshold - 5)
                print(f"[main] rat v_threshold -> {rat_params_box[0].v_threshold}")
            elif key == ord("'") and rat_params_box[0] is not None:
                rat_params_box[0].v_threshold = min(250, rat_params_box[0].v_threshold + 5)
                print(f"[main] rat v_threshold -> {rat_params_box[0].v_threshold}")
            elif key == ord('<') and rat_params_box[0] is not None:
                rat_params_box[0].min_area = max(10, rat_params_box[0].min_area - 10)
                print(f"[main] rat min_area -> {rat_params_box[0].min_area}")
            elif key == ord('>') and rat_params_box[0] is not None:
                rat_params_box[0].min_area += 10
                print(f"[main] rat min_area -> {rat_params_box[0].min_area}")
            elif key == ord('b') and rat_params_box[0] is not None:
                rat_params_box[0].max_area = max(rat_params_box[0].min_area + 10,
                                                 rat_params_box[0].max_area - 10)
                print(f"[main] rat max_area -> {rat_params_box[0].max_area}")
            elif key == ord('B') and rat_params_box[0] is not None:
                rat_params_box[0].max_area += 10
                print(f"[main] rat max_area -> {rat_params_box[0].max_area}")

            elif key == ord('s'):
                save_idx += 1
                fname = f"session_{save_idx:03d}.png"
                cv.imwrite(fname, disp)
                print(f"[main] saved {fname}")

    finally:
        with state.lock:
            state.running = False
        # Robot stop + disconnect are handled by the control thread
        # itself (it sees state.running = False and cleans up).
        cv.destroyAllWindows()
        # Allow the control thread time to send its final stop+disconnect
        # over BLE before we exit.
        ctrl_thread.join(timeout=3.0)
        if 'proc_thread' in locals():
            proc_thread.join(timeout=1.0)
        grabber.stop()
        arduino.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
