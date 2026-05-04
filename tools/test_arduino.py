"""
tools/test_arduino.py
Standalone REPL for testing the Arduino digital-I/O bridge in isolation.
Run this BEFORE integrating with the rest of the rig to verify wiring.

Usage (from project root):
    python -m tools.test_arduino                  # auto-detect port
    python -m tools.test_arduino --port COM4      # use specific port

Commands at the prompt:
    pulse <pin>      e.g. "pulse 3"     500 ms HIGH on D3, then LOW
    toggle           toggle D13
    ping             liveness check (returns True/False)
    burst <pin> <n>  fire n pulses on the same pin, ~50 ms apart
                     (useful to confirm queue + Arduino keep up)
    sleep <s>        pause for N seconds (useful when scripting input)
    quit / q         exit

Tips:
    - Pulse pins must be in 2..12. D13 is toggle-only (use the 'toggle' cmd).
    - First connection takes ~2 seconds while the Arduino auto-resets.
"""

import argparse
import sys
import time
import traceback

sys.path.insert(0, ".")

import config
from arduino_io import connect


def cmd_pulse(io, args):
    if len(args) != 1:
        print("usage: pulse <pin>")
        return
    io.pulse(int(args[0]))


def cmd_toggle(io, _args):
    io.toggle(13)


def cmd_ping(io, _args):
    print("PONG" if io.ping() else "(no response)")


def cmd_burst(io, args):
    if len(args) != 2:
        print("usage: burst <pin> <n>")
        return
    pin = int(args[0])
    n = int(args[1])
    for i in range(n):
        io.pulse(pin)
        time.sleep(0.05)
    print(f"queued {n} pulses on D{pin}")


def cmd_sleep(_io, args):
    if len(args) != 1:
        print("usage: sleep <seconds>")
        return
    time.sleep(float(args[0]))


def cmd_help(_io, _args):
    print(__doc__)


COMMANDS = {
    "pulse": cmd_pulse,
    "toggle": cmd_toggle,
    "ping": cmd_ping,
    "burst": cmd_burst,
    "sleep": cmd_sleep,
    "help": cmd_help,
    "?": cmd_help,
}


def repl(io) -> None:
    print("\nArduino debug console. Type 'help' for commands, 'quit' to exit.\n")
    while True:
        try:
            line = input("ard> ").strip()
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
            handler(io, args)
        except Exception:
            traceback.print_exc()


def main() -> int:
    p = argparse.ArgumentParser(description="Arduino debug REPL")
    p.add_argument("--port", type=str, default=config.ARDUINO_PORT)
    args = p.parse_args()

    io = connect(
        port=args.port,
        baud=config.ARDUINO_BAUD,
        boot_wait_s=config.ARDUINO_BOOT_WAIT_S,
        ping_timeout_s=config.ARDUINO_PING_TIMEOUT_S,
    )
    if io is None:
        print("[error] no Arduino found")
        return 1

    try:
        repl(io)
    finally:
        io.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
