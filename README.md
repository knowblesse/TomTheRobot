# TomTheRobot
Closed-loop dual-tracking and RVR auto-pilot

## Setup

### 0. (Optional) Install virtual environment manager.
- Go to [miniforge github page](https://github.com/conda-forge/miniforge), releases, and download the appropriate installer for your OS and architecture. For example, on Windows with an Intel/AMD CPU, download `Miniforge3-Windows-x86_64.exe`.
- Run the installer and follow the prompts.
- After the installation, run **Miniforge Prompt** from the Start menu (Windows) and create a new environment for the project using the command below.
```
conda create -n ttr python=3.12
```
- Then, activate the environment:
```
conda activate ttr
```

### 1. Install dependencies
- Install the required Python packages using pip. Make sure to do this in the virtual environment you just created and activated.
```
pip install opencv-python numpy filterpy spherov2 bleak pyserial
```

### 2. Pair the RVR+ via Bluetooth
- On Windows, also pair the RVR+ once via system Bluetooth settings before first run (Settings → Bluetooth → Add device → look for `RV-XXXX`).

### 3. Check camera index
- This project is configured for a PC's primary webcam at `CAMERA_INDEX = 0` in `config.py`. If you have multiple cameras and the wrong one opens, change the index.
- To find which index is which, run the following one-liner in your activated environment. It prints `True` for each working camera:
```
python -c "import cv2; [print(i, cv2.VideoCapture(i, cv2.CAP_DSHOW).isOpened()) for i in range(5)]"
```

### 4. Find Arduino COM port
- Plug in the Arduino, open **Devices and Printers** (Start → search "Devices and Printers"), and look for an entry like "Arduino Uno (COM4)" or "USB Serial Device (COM4)" under **Unspecified** or **Other devices**. Note the COM number.
- Update `ARDUINO_PORT = "COM4"` in `config.py` to match your COM number. (Set it to `None` to auto-detect at startup instead.)

### 5. Upload the Arduino sketch
- Install the [Arduino IDE](https://www.arduino.cc/en/software) if you don't already have it.
- Open `arduino/rattracker_io.ino` in the Arduino IDE.
- Select your board: **Tools → Board → Arduino Uno** (or whichever you have).
- Select the port: **Tools → Port → COM4** (the one you found in step 4).
- Click **Upload** (the right-arrow button). Wait until "Done uploading" appears.

## Running

### Open Miniforge Prompt and navigate to the project folder
```
cd "C:\path\to\TomTheRobot"
```
(Replace the path with wherever you cloned/saved the project.)

### Launch the main program
```
python main.py
```

### First-run calibration prompts
On first run, you will be asked to:
1. **Draw the arena polygon** — left-click to add vertices around the arena, ENTER to finish, ESC to cancel. Right-click or `z` to undo a vertex.
2. **Sample the red marker** — drag a rectangle over the red marker on the robot, press ENTER.
3. **Sample the rat darkness** — drag a rectangle over the rat's dark hood (or a dark stand-in), press ENTER.

Press `w` after the live view appears to save the polygon, color sample, and threshold to `last_calibration.json`. Subsequent runs will auto-load these and skip the prompts.

After calibration, the script will:
- Connect to the Arduino (warning printed if missing — rest of program still runs)
- Connect to the RVR+ over Bluetooth (this can take ~15 seconds; retries up to 3 times)
- Prompt: *"place robot facing the TOP of the image, then press ENTER"* — physically point the robot's headlights toward the top of the camera image, then press ENTER in the terminal

### Quitting
Press `q` or ESC in the live window. The robot stops, BLE disconnects, the active session is finalized, files are flushed.

### Configurations
For tuning constants in `config.py`, see [docs/CONFIG_GUIDE.md](docs/CONFIG_GUIDE.md).

## Key Bindings
All keys are pressed in the live OpenCV window (not the terminal).

```
calibrate:  m=polygon  c=red  r=rat  h=reset_aim  w=save
control:    t=toggle_chase   space=robot_disable/enable   q/ESC=quit
session:    1=start  2=pause  3=halt
red tune:   [/] range    ,/. min_area    n/N max_area
rat tune:   ;/' V_thr    </> min_area    b/B max_area
general:    s=screenshot
```

The same panel is shown at the bottom of the live window during operation.

## Project Overview

### 1. Tracking
Uses OpenCV for tracking the robot's position and the rat's position.
- The robot is detected by blob color (red, in LAB color space for lighting robustness) and area.
- The rat is detected by blob darkness (V channel of HSV) and area.
- Kalman filters smooth the tracking and estimate velocity for both objects.
- Occlusion detection prevents identity confusion when the two blobs visually merge.

### 2. Communications
- `comms/rvr_bluetooth.py` — connects to and sends drive commands to the RVR+ via Bluetooth.
- `arduino_io.py` — sends digital I/O pulses to the Arduino via USB serial. Used for synchronization with external recording systems.

### 3. Control
- `controller/chase.py` — implements the chase-the-rat logic with hysteretic near/far behavior, stuck detection, and BLE heartbeats.

### 4. Session logging
Per-session positional CSV plus periodic image snapshots in `./sessions/`.

Triggered by keys `1` (start), `2` (pause), `3` (halt). Each session creates a fresh timestamped folder:
```
sessions/
  20260504_142301_session_01/
    positions.csv          # one row per frame, including paused frames
    snapshots/
      000042.jpg           # filename = frame_idx
      001234.jpg
      ...
```

CSV columns: `timestamp, wall_time, frame_idx, session_id, session_state, rat_x, rat_y, rat_status, robot_x, robot_y, robot_status, occlusion, last_cmd_speed, last_cmd_heading, last_cmd_stop`

`session_state` is `running` or `paused`. Pause stops the chase but continues logging; press `1` again to resume the same session.

---

## Troubleshooting

The project has standalone test scripts for diagnosing each subsystem in isolation. Run these from the project folder, in your activated environment.

### Test the RVR+ Bluetooth connection
- Power on the RVR+, then run:
```
python -m comms.debug_console
```
- Suggested test sequence:
```
forward 1            # robot rolls forward 1 second, then stops
stop
heading 90           # robot turns to face 90°
speed 60             # robot starts driving at heading 90
stop
reset                # current orientation becomes the new "0°"
square 50 1          # drives a rough square (one side per second at speed 50)
quit
```
- If `forward 1` does not produce motion: check that the robot is not in sleep mode (tap it), that Bluetooth is paired, and that no other Sphero app is connected to the robot.

### Test the Arduino digital I/O
```
python -m tools.test_arduino
```
At the `ard>` prompt:
```
ping                 # should return PONG
pulse 3              # 500 ms HIGH on D3
toggle               # flip D13 latched state
quit
```

### Test the camera capture
```
python -m tools.test_capture
```
Shows a live preview with FPS, lag, and frame index. Should report ~30 fps with low lag.

### Test the detection (color sampling and tuning)
```
python -m tools.test_detection
```
Lets you draw the arena, sample the red marker and rat, and tune detection parameters live without involving the robot.

### Test the tracker (Kalman + occlusion handling)
```
python -m tools.test_tracker
```
Synthetic motion test — no camera, no robot. Press `o` to force an occlusion, `d` to force a detection dropout, and watch the filter handle them.