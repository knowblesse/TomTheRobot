# Configuration Guide

This guide explains what each value in `config.py` does and when you might want to change it. The values shipped in `config.py` are tuned for a specific setup (Long-Evans rat, RVR+ robot, 640×480 webcam, white arena floor). If your setup differs, this is the file to edit.

> **Tip:** make small changes (one value at a time), restart the program, and test. Big changes in many values at once make problems hard to trace.

---

## Hardware

These describe the camera and the Arduino link. Change them when your physical setup changes (different camera, different USB port).

### Camera

```
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_FOURCC = 'MJPG'
```

- **`CAMERA_INDEX`** — which camera the script opens. `0` is usually the only or main camera. If you have two cameras and the wrong one opens, change to `1`. Use the one-liner from the main README to check which index is which.
- **`CAMERA_WIDTH` / `CAMERA_HEIGHT`** — image size in pixels. Larger values give finer detection but cost more CPU. The defaults (640×480) work well at 30 fps. Most laptops can also do 1280×720 but you may drop frames.
- **`CAMERA_FPS`** — frames per second. 30 is standard for USB webcams. The script does not need exactly 30; whatever your camera delivers is what you get.
- **`CAMERA_FOURCC`** — frame format. `'MJPG'` (compressed) is needed to hit 30 fps over USB 2.0. Don't change unless the camera fails to open.

### Arduino

```
ARDUINO_PORT: str | None = "COM4"
ARDUINO_BAUD = 115200
ARDUINO_PING_TIMEOUT_S = 1.5
ARDUINO_BOOT_WAIT_S = 2.0
ARDUINO_PIN_CHASE_START = 2
```

- **`ARDUINO_PORT`** — the COM port name. Set to whatever Windows shows in Devices and Printers (e.g. `"COM4"`). Set to `None` to auto-detect every time the script starts (slightly slower).
- **`ARDUINO_BAUD`** — communication speed. Must match the Arduino sketch (it's `115200` there too). Don't change.
- **`ARDUINO_PING_TIMEOUT_S`** — how long to wait for the Arduino to reply during the startup handshake. If startup fails with "no response," try increasing to `3.0`.
- **`ARDUINO_BOOT_WAIT_S`** — how long to wait for the Arduino to finish its boot sequence after the port is opened. The Arduino auto-restarts when the PC opens its serial port; this gives it time to come back up. Increase if the first command after startup is missed.
- **`ARDUINO_PIN_CHASE_START`** — which pin sends a 500 ms pulse when chase mode is toggled on. Default is D2. Change if you wire a different pin to your recording system.

---

## Detection

This is how the script finds the robot and the rat in each frame. The defaults are tuned for our specific marker sizes and lighting; **you'll usually re-tune these when you change marker, rat, or lighting**.

### Robot (red marker)

```
ROBOT_COLOR_RANGE = 30
ROBOT_MIN_AREA_PX = 500
ROBOT_MAX_AREA_PX = 2000
USE_LAB_FOR_RED = True
```

- **`ROBOT_COLOR_RANGE`** — how strict the color matching is, in LAB color units. Lower = stricter (only very-similar reds count); higher = looser. If the robot is sometimes lost when it crosses lighting boundaries, increase. If a red object somewhere else in the room gets detected as the robot, decrease.
- **`ROBOT_MIN_AREA_PX`** — minimum blob size in pixels to be considered the robot. If the robot is sometimes lost, lower this. If a red speck of noise is sometimes detected as the robot, raise this. The robot's real size on screen is shown in the live status panel ("size=").
- **`ROBOT_MAX_AREA_PX`** — maximum blob size. If a large red region (e.g. a piece of clothing in frame) is being detected as the robot, lower this.
- **`USE_LAB_FOR_RED`** — whether to threshold red in LAB color space. Leave `True`. LAB handles uneven lighting much better than the raw camera color (BGR).

### Rat (dark hood)

```
RAT_DARKNESS_THRESHOLD_DEFAULT = 36
RAT_MIN_AREA_PX = 70
RAT_MAX_AREA_PX = 470
```

- **`RAT_DARKNESS_THRESHOLD_DEFAULT`** — how dark a pixel must be to count as "rat." Lower = stricter (only very-dark pixels qualify); higher = looser. Used as a starting point only; you re-sample with the `r` key, which usually overrides this. If the rat detection misses lighter parts of the hood, raise. If shadows on the floor get detected as rat, lower.
- **`RAT_MIN_AREA_PX`** — minimum blob size to count as a rat. Important for rejecting small dark specks from the floor or noise. Raise if a dark mark on the floor is detected as rat. Lower if the rat is small or partly occluded and gets missed.
- **`RAT_MAX_AREA_PX`** — maximum blob size to count as a rat. Useful for rejecting huge dark regions (camera shadow, dark equipment). Lower if dark non-rat objects are being picked up. Raise if a posture change (rat stretches out) makes the hood look bigger than expected.

### Morphology (image cleanup)

```
MORPH_OPEN_KERNEL = 3
MORPH_CLOSE_KERNEL = 15
```

- **`MORPH_OPEN_KERNEL`** — removes small specks of noise. Higher value = more aggressive noise cleanup, but also chips away at small real targets. If the rat sometimes flickers in and out, lower this. If small noise specks are detected, raise.
- **`MORPH_CLOSE_KERNEL`** — fills small holes inside detected blobs. Higher = larger holes are filled. If the rat or robot is sometimes split into two blobs, raise. If two nearby objects merge into one blob, lower.

---

## Tracking (Kalman Filter)

A Kalman filter takes the noisy detection from each frame and produces a smoothed position and velocity estimate. It also predicts where the object should be when detection briefly fails.

The on-screen marker in the live view is the *Kalman-smoothed* position. The small dot is the *raw detection* (no smoothing).

```
KALMAN_PROCESS_NOISE_RAT = 500.0
KALMAN_PROCESS_NOISE_ROBOT = 250.0
KALMAN_MEASUREMENT_NOISE = 3.0

MAX_FRAMES_LOST = 10
ASSOCIATION_GATE_PX = 80
OCCLUSION_DISTANCE_PX = 60
```

- **`KALMAN_PROCESS_NOISE_RAT`** — how much we trust the rat's measurements vs. the predicted motion. Higher = trust measurements more, less smoothing. Lower = trust the prediction more, smoother but laggy. **If the rat cross follows the rat slowly, increase. If the rat cross is jittery and shaky, decrease.**
- **`KALMAN_PROCESS_NOISE_ROBOT`** — same idea for the robot. The robot moves more predictably than the rat, so this is usually lower than the rat's value. **If the robot cross lags behind the actual robot, increase. If the robot cross hops around when the robot is stationary, decrease.**
- **`KALMAN_MEASUREMENT_NOISE`** — how noisy you think the detection itself is, in pixels. Lower = trust each measurement very precisely. Higher = average across many frames. Default `3.0` says "the centroid is accurate to about 3 pixels." Rarely needs changing.
- **`MAX_FRAMES_LOST`** — how many frames in a row the detector can miss before declaring the object "lost" and resetting the filter. Default `10` ≈ 0.3 s at 30 fps. Higher = the script tolerates longer occlusions but takes longer to recover when the object reappears far away. Lower = recovers quickly but gives up on brief occlusions.
- **`ASSOCIATION_GATE_PX`** — when a new detection arrives, it's accepted only if it's within this many pixels of the predicted position. Otherwise it's rejected as a false detection. Lower = stricter (rejects more, more robust to noise). Higher = looser (accepts more, more tolerant of fast motion or long occlusions). If real detections of fast-moving objects are being rejected, raise. If false detections are being accepted, lower.
- **`OCCLUSION_DISTANCE_PX`** — when the predicted positions of rat and robot are within this many pixels of each other and the detector returns only one blob, the script declares an "occlusion" and stops trusting the single detection. Lower = only flags occlusion when objects are very close. Higher = flags occlusion sooner. Adjust based on your robot + rat sizes.

---

## Control (Chase Logic)

This is how the robot decides where to drive.

```
STOP_DISTANCE_PX = 50
RESUME_DISTANCE_PX = 80
ROBOT_DEFAULT_SPEED = 80
ROBOT_MAX_SPEED = 100
HEADING_DEADBAND_DEG = 5
HEARTBEAT_INTERVAL_S = 0.5
```

- **`STOP_DISTANCE_PX`** — when the robot gets this close to the rat (in pixels), it stops driving but keeps rotating to face the rat. **Lower = robot gets closer to the rat before stopping. Higher = robot keeps a larger distance.**
- **`RESUME_DISTANCE_PX`** — once stopped near the rat, the robot starts driving again only when the rat moves farther than this. Must be larger than `STOP_DISTANCE_PX` — the gap between them prevents the robot from oscillating between drive and stop. **Larger gap = more stable behavior. Smaller gap = more responsive but may oscillate.**
- **`ROBOT_DEFAULT_SPEED`** — the speed used during chase, on a 0–255 scale. Default 80. **Higher = robot chases faster. Lower = slower, gentler.**
- **`ROBOT_MAX_SPEED`** — safety cap. The script never sends a speed higher than this even if other code asks for more. Don't raise above ~150 unless you know what you're doing.
- **`HEADING_DEADBAND_DEG`** — the smallest direction change (in degrees) that triggers a new command. Below this, the robot keeps its current direction. **Higher = robot ignores small changes (smoother but coarser). Lower = robot reacts to every wiggle (more responsive but spammier).**
- **`HEARTBEAT_INTERVAL_S`** — how often the script re-sends the current command to the robot, even if nothing changed, as a safety mechanism. If the program crashes, the robot's own internal timer will eventually stop it. Don't usually change.

### Stuck detection

```
STUCK_VELOCITY_PX_S = 30
STUCK_DURATION_S = 3.0
```

- **`STUCK_VELOCITY_PX_S`** — if the robot's observed speed (from the camera) stays below this many pixels per second while it's being told to drive, it counts as "stuck." Higher = stricter (more things are flagged as stuck). Lower = more tolerant (slow motion is allowed).
- **`STUCK_DURATION_S`** — how long the robot must be stuck before the script gives up and stops it. Default 3 seconds. Lower = quicker giveup. Higher = the robot tries harder to escape before stopping.

### Initial heading

```
INITIAL_IMU_OFFSET_DEG = 0.0
```

- **`INITIAL_IMU_OFFSET_DEG`** — the direction (in camera-frame degrees) the robot is physically pointed when the program starts. Convention: `0` = top of image, `90` = right of image, `180` = bottom, `270` = left. **Change this if you want to place the robot facing a different direction at startup.** The startup prompt will tell you which direction to point the robot based on this value.

---

## Logging

```
LOG_DIR = "./sessions"
CSV_FLUSH_EVERY_N = 30
SNAPSHOT_EVERY_S = 10.0
```

- **`LOG_DIR`** — where session folders are saved. Each session gets its own timestamped subfolder.
- **`CSV_FLUSH_EVERY_N`** — how often (in rows) the position log is written to disk. Higher = less disk activity but more data lost if the program crashes. Lower = more disk writes but less data lost. Default 30 ≈ once per second.
- **`SNAPSHOT_EVERY_S`** — how often during a session a still image of the camera view is saved. Default 10 seconds. Lower = more snapshots, more disk space. Higher = fewer snapshots.

---

## Common Problems → Which Knob to Turn

| Symptom | Try |
|---------|-----|
| Robot cross lags behind the moving robot | Raise `KALMAN_PROCESS_NOISE_ROBOT` |
| Rat cross lags behind the moving rat | Raise `KALMAN_PROCESS_NOISE_RAT` |
| Cross is jittery / shaky when target is still | Lower the corresponding `KALMAN_PROCESS_NOISE_*` |
| Detection misses the rat / robot sometimes | Lower the `MIN_AREA_PX` or raise color/darkness range |
| False detection on background noise | Raise the `MIN_AREA_PX` or tighten color/darkness range |
| Detection picks up a wrong huge region | Lower the `MAX_AREA_PX` |
| Robot oscillates between driving and stopping near the rat | Increase the gap between `STOP_DISTANCE_PX` and `RESUME_DISTANCE_PX` |
| Robot gets too close / not close enough to the rat | Adjust `STOP_DISTANCE_PX` |
| Robot drives too slowly / too fast | Adjust `ROBOT_DEFAULT_SPEED` |
| Robot is flagged stuck during normal slow motion | Lower `STUCK_VELOCITY_PX_S`, raise `STUCK_DURATION_S` |
| Robot drives in the wrong direction at startup | Change `INITIAL_IMU_OFFSET_DEG` to match where the robot is pointed |
| Arduino missed the first command on startup | Raise `ARDUINO_BOOT_WAIT_S` to 3.0 |