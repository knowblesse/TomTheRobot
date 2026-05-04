# RatTracker

Closed-loop tracking of a Long-Evans rat with Sphero RVR+ control.

## Status

In progress. Currently implemented:

- `config.py` — all tunable constants
- `world_state.py` — shared dataclasses for inter-thread state
- `comms/` — robot communication layer
  - `interface.py` — `RobotInterface` ABC
  - `rvr_bluetooth.py` — `spherov2` over Bluetooth (real RVR+)
  - `mock_robot.py` — no-hardware mock that just logs commands
  - `debug_console.py` — standalone REPL for testing comms in isolation

Not yet implemented:

- `capture.py`, `detection.py`, `calibration.py`, `tracker.py`,
  `controller/`, `logger.py`, `display.py`, `main.py`

## Setup

```
conda create -n rattracker python=3.12
conda activate rattracker
pip install -r requirements.txt
```

On Windows, also pair the RVR+ once via system Bluetooth settings before
first run (Settings → Bluetooth → Add device → look for `RV-XXXX`).

## How to test the comms layer

This is the first thing to verify, before the vision pipeline exists.

### With no hardware (sanity check)

```
python -m comms.debug_console --mock
```

You should get a `rvr>` prompt. Type `help`, then `forward 1`, `stop`, `quit`.

### With real RVR+

Power on the RVR+ and:

```
python -m comms.debug_console
```

Suggested test sequence:

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

If `forward 1` does not produce motion: check that the robot is not in
sleep mode (tap it), that Bluetooth is paired, and that no other Sphero
app is connected to the robot at the same time.

## Project layout

```
rattracker/
├── config.py              # all tunable constants
├── world_state.py         # shared dataclasses
├── requirements.txt
├── comms/
│   ├── __init__.py
│   ├── interface.py       # RobotInterface ABC
│   ├── rvr_bluetooth.py   # spherov2 implementation
│   ├── mock_robot.py      # no-hardware mock
│   └── debug_console.py   # standalone REPL
├── controller/            # (coming next)
├── tools/                 # (coming next: detection test scripts etc.)
└── sessions/              # log output goes here
```
