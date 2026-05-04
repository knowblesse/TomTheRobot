"""
config.py
All tunable constants for the rat tracker + RVR+ closed-loop system.
Edit values here; do not put magic numbers in other modules.
"""

###############################################################
#                       Hardware                              #
###############################################################
CAMERA_INDEX = 1               # USB webcam (index 0 is the built-in laptop cam)
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30                # request from camera; actual may differ
CAMERA_FOURCC = 'MJPG'         # USB 2.0 bandwidth; needed for 30fps at 640x480

# Arduino digital I/O over USB serial.
# Set to a specific COM port (e.g. "COM4") to override; None = auto-detect.
# Auto-detect scans available ports and looks for one that responds to PING.
ARDUINO_PORT: str | None = None
ARDUINO_BAUD = 115200
ARDUINO_PING_TIMEOUT_S = 1.5    # max time to wait for PONG during handshake
ARDUINO_BOOT_WAIT_S = 2.0       # Arduino auto-resets when port opens; wait for boot
# Pin assignments. Pulse pins fire HIGH for 500 ms then LOW. D13 latches (toggle).
ARDUINO_PIN_CHASE_START = 2     # pulse on chase-start

###############################################################
#                       Detection                             #
###############################################################
# Red robot marker (tuned: marker measures ~1230 px area)
ROBOT_COLOR_RANGE = 30         # +/- range around sampled LAB color
ROBOT_MIN_AREA_PX = 500
ROBOT_MAX_AREA_PX = 2000
USE_LAB_FOR_RED = True         # True: threshold in LAB (lighting-robust); False: BGR

# Long-Evans rat hood (tuned: hood measures ~220 px area)
RAT_DARKNESS_THRESHOLD_DEFAULT = 36   # V channel cutoff
RAT_MIN_AREA_PX = 120
RAT_MAX_AREA_PX = 470

# Morphology
MORPH_OPEN_KERNEL = 3          # remove speckle noise
MORPH_CLOSE_KERNEL = 15        # fill holes in marker

###############################################################
#                       Tracking                              #
###############################################################
KALMAN_PROCESS_NOISE_RAT = 500.0    # high — rats are erratic; trust measurements
KALMAN_PROCESS_NOISE_ROBOT = 250.0   # moderate — robot moves predictably under our cmd
KALMAN_MEASUREMENT_NOISE = 3.0      # pixels (lower = trust measurements more)

MAX_FRAMES_LOST = 10                # ~0.5s at 20fps before declaring "lost"
ASSOCIATION_GATE_PX = 80            # max distance for measurement-to-track
OCCLUSION_DISTANCE_PX = 60          # blobs closer than this = occluded

###############################################################
#                       Control                               #
###############################################################
STOP_DISTANCE_PX = 50               # CHASE: stop driving when rat closer than this
RESUME_DISTANCE_PX = 80             # CHASE: resume driving when rat farther than this
                                    # (hysteresis gap between STOP and RESUME prevents
                                    # oscillation when rat hovers near the threshold)
ROBOT_DEFAULT_SPEED = 80            # CHASE: forward speed (0-255)
ROBOT_MAX_SPEED = 100               # safety cap (we never exceed this)
HEADING_DEADBAND_DEG = 5            # don't re-issue command if Δheading < this
HEARTBEAT_INTERVAL_S = 0.5          # re-send command at this rate as deadman

# Stuck detection: commanded speed > 0 but observed velocity nearly zero
STUCK_VELOCITY_PX_S = 30            # below this observed speed = "not moving"
STUCK_DURATION_S = 3.0              # how long to be stuck before alarming

# IMU-to-camera offset, in camera-frame degrees.
# Convention: 0=image-up, 90=image-right, 180=image-down, 270=image-left.
# Whatever direction the robot is *physically pointed* at reset_aim time
# is the value to put here. Default is 0 (robot faces image-up).
INITIAL_IMU_OFFSET_DEG = 0.0

###############################################################
#                       Logging                               #
###############################################################
LOG_DIR = "./sessions"
SAVE_VIDEO_DEFAULT = False
DISPLAY_DEFAULT = True
CSV_FLUSH_EVERY_N = 30              # flush to disk this often (~1s at 30fps)

###############################################################
#                       Keys (in main GUI)                    #
###############################################################
KEY_QUIT = ord('q')
KEY_STOP_ROBOT = ord(' ')           # space: emergency stop
KEY_TOGGLE_FOLLOW = ord('t')        # t: toggle "follow rat" mode
