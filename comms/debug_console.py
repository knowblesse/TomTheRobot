"""
comms/debug_console.py
Standalone REPL for testing the RVR+ Bluetooth connection in isolation.
Run this BEFORE integrating with the vision pipeline.

Usage (from project root):
    python -m comms.debug_console               # connects to real RVR+
    python -m comms.debug_console --mock        # uses MockRobot, no hardware

Commands at the prompt:
    drive <heading> <speed>    e.g. "drive 0 80"     drive forward at speed 80
    heading <0-359>            e.g. "heading 90"     turn to face right
    speed <0-255>              e.g. "speed 60"       set speed without changing heading
    stop                       stop motion
    reset                      reset_aim (current orientation becomes 0°)
    square <speed> <side_s>    drive a rough square: forward, right, back, left
    forward <duration_s>       drive forward for N seconds at speed 60, then stop
    spin                       spin in place: heading 0 -> 90 -> 180 -> 270 -> 0
    sleep <s>                  pause for N seconds (useful when scripting)
    help                       this list
    quit / q                   exit (always stops the robot first)

Tips:
    - In RVR's heading convention: 0 = forward, 90 = right, 180 = back, 270 = left.
    - First connect can take ~5s. Be patient.
    - If the robot drifts in an unexpected direction, run "reset" while it's
      pointing the way you want "forward" to be.
"""

import argparse
import sys
import time
import traceback

from .interface import RobotInterface
from .mock_robot import MockRobot


def make_robot(use_mock: bool) -> RobotInterface:
    if use_mock:
        return MockRobot(verbose=True)
    # Imported here so a missing spherov2 install doesn't break --mock mode
    from .rvr_bluetooth import RvrBluetooth
    return RvrBluetooth(timeout_s=10.0)


def cmd_drive(robot, args):
    if len(args) != 2:
        print("usage: drive <heading 0-359> <speed 0-255>")
        return
    heading = int(args[0])
    speed = int(args[1])
    robot.set_heading(heading)
    robot.set_speed(speed)


def cmd_heading(robot, args):
    if len(args) != 1:
        print("usage: heading <0-359>")
        return
    robot.set_heading(int(args[0]))


def cmd_speed(robot, args):
    if len(args) != 1:
        print("usage: speed <0-255>")
        return
    robot.set_speed(int(args[0]))


def cmd_stop(robot, _args):
    robot.stop()


def cmd_reset(robot, _args):
    robot.reset_aim()


def cmd_square(robot, args):
    if len(args) != 2:
        print("usage: square <speed> <side_seconds>")
        return
    speed = int(args[0])
    side_s = float(args[1])
    for h in (0, 90, 180, 270):
        print(f"  segment heading={h} speed={speed} for {side_s}s")
        robot.set_heading(h)
        robot.set_speed(speed)
        time.sleep(side_s)
    robot.stop()


def cmd_forward(robot, args):
    if len(args) != 1:
        print("usage: forward <duration_seconds>")
        return
    dur = float(args[0])
    robot.set_heading(0)
    robot.set_speed(60)
    time.sleep(dur)
    robot.stop()


def cmd_spin(robot, _args):
    for h in (0, 90, 180, 270, 0):
        print(f"  heading -> {h}")
        robot.set_heading(h)
        time.sleep(1.5)


def cmd_sleep(_robot, args):
    if len(args) != 1:
        print("usage: sleep <seconds>")
        return
    time.sleep(float(args[0]))


def cmd_help(_robot, _args):
    print(__doc__)


COMMANDS = {
    "drive": cmd_drive,
    "heading": cmd_heading,
    "speed": cmd_speed,
    "stop": cmd_stop,
    "reset": cmd_reset,
    "square": cmd_square,
    "forward": cmd_forward,
    "spin": cmd_spin,
    "sleep": cmd_sleep,
    "help": cmd_help,
    "?": cmd_help,
}


def repl(robot: RobotInterface) -> None:
    print("\nDebug console. Type 'help' for commands, 'quit' to exit.\n")
    while True:
        try:
            line = input("rvr> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in ("quit", "q", "exit"):
            break

        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        handler = COMMANDS.get(cmd)
        if handler is None:
            print(f"unknown command: {cmd!r}; type 'help'")
            continue
        try:
            handler(robot, args)
        except Exception:
            traceback.print_exc()


def main() -> int:
    p = argparse.ArgumentParser(description="RVR+ debug console")
    p.add_argument("--mock", action="store_true", help="Use MockRobot (no hardware)")
    args = p.parse_args()

    robot = make_robot(args.mock)
    try:
        robot.connect()
    except Exception as e:
        print(f"[error] connect failed: {e}")
        return 1

    try:
        repl(robot)
    finally:
        try:
            robot.stop()
        except Exception:
            pass
        robot.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
