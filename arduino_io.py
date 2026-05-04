"""
arduino_io.py
Serial digital-I/O bridge to the Arduino sketch in arduino/rattracker_io.ino.

Responsibilities:
  - Find and open the Arduino's serial port (config or auto-detect).
  - Confirm the sketch is alive via a PING/PONG handshake at startup.
  - Provide a non-blocking pulse(pin) / toggle(pin) API; calls return
    immediately, a worker thread drains a queue and writes to serial.

Design notes:
  - We never block the closed-loop control path. The chase controller
    calls io.pulse(...) which just enqueues; the worker thread does the
    actual write at its own pace.
  - We do not require ACK from the Arduino for individual commands
    (per spec: skip ACK). The PING/PONG handshake at startup is the
    only confirmation that the link is alive.
  - If the Arduino is not present or fails to respond, the higher-level
    code can still construct a NoOpArduinoIO that silently swallows
    everything, so the rest of the rig still runs.

Wire protocol (matches arduino/rattracker_io.ino):
    "PING\\n"         -> "PONG\\n"
    "D<n>\\n"         -> 500ms pulse on pin n (n in 2..12)
    "D13\\n"          -> toggle pin 13
"""

from __future__ import annotations

import queue
import threading
import time
from typing import List, Optional

import serial
from serial.tools import list_ports


# -----------------------------------------------------------------
#                    Auto-detect helpers
# -----------------------------------------------------------------

def _candidate_ports() -> List[str]:
    """Return likely Arduino USB ports. We don't filter by VID/PID
    because Arduino clones use various USB-Serial chips (CH340, FT232,
    CP210x). Instead, return all ports and let PING decide."""
    return [p.device for p in list_ports.comports()]


def _try_handshake(port: str, baud: int, boot_wait_s: float,
                   ping_timeout_s: float) -> Optional[serial.Serial]:
    """Open the port, wait for boot, send PING, and confirm PONG.
    Returns an open serial handle on success, None on any failure."""
    try:
        ser = serial.Serial(port, baud, timeout=ping_timeout_s)
    except Exception:
        return None

    try:
        # Arduino auto-resets when the port opens; give it time to boot.
        time.sleep(boot_wait_s)
        # Drain any boot-time chatter (e.g. our sketch prints "READY")
        ser.reset_input_buffer()

        ser.write(b"PING\n")
        ser.flush()
        # readline blocks up to `timeout` set above
        line = ser.readline().decode(errors="ignore").strip()
        if line == "PONG":
            return ser
        # Some boards may print "READY" first if the buffer wasn't cleared
        # in time; allow one more read.
        line = ser.readline().decode(errors="ignore").strip()
        if line == "PONG":
            return ser
    except Exception:
        pass

    try:
        ser.close()
    except Exception:
        pass
    return None


# -----------------------------------------------------------------
#                    Public API
# -----------------------------------------------------------------

class ArduinoIO:
    """Serial-backed digital I/O. Use connect() then pulse()/toggle()."""

    def __init__(self, port: str, ser: serial.Serial):
        self._port = port
        self._ser = ser
        self._q: queue.Queue[str] = queue.Queue(maxsize=64)
        self._stop_evt = threading.Event()
        self._thread = threading.Thread(
            target=self._worker, name="arduino-io", daemon=True
        )
        self._thread.start()
        print(f"[arduino] connected on {port}")

    # ---- public ----

    @property
    def port(self) -> str:
        return self._port

    def pulse(self, pin: int) -> None:
        """Fire-and-forget: request a 500 ms HIGH pulse on the given pin.
        Pin must be in 2..12 (D13 is toggle; use toggle() for it)."""
        if pin < 2 or pin > 12:
            raise ValueError(f"pulse pin must be 2..12, got {pin}")
        self._enqueue(f"D{pin}")

    def toggle(self, pin: int = 13) -> None:
        """Fire-and-forget: flip the latched state of D13."""
        if pin != 13:
            raise ValueError("toggle is only supported on D13")
        self._enqueue("D13")

    def ping(self) -> bool:
        """Synchronous liveness check. Sends PING, waits up to 1s for PONG.
        Bypasses the queue. Use sparingly — blocks the caller. Returns
        True if the Arduino is alive and responsive."""
        try:
            # Drain queue briefly so our PING isn't queued behind pulses
            self._ser.reset_input_buffer()
            self._ser.write(b"PING\n")
            self._ser.flush()
            old_timeout = self._ser.timeout
            self._ser.timeout = 1.0
            line = self._ser.readline().decode(errors="ignore").strip()
            self._ser.timeout = old_timeout
            return line == "PONG"
        except Exception:
            return False

    def close(self) -> None:
        self._stop_evt.set()
        self._thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass
        print("[arduino] disconnected")

    # ---- internals ----

    def _enqueue(self, msg: str) -> None:
        try:
            self._q.put_nowait(msg)
        except queue.Full:
            print(f"[arduino] queue full; dropping '{msg}'")

    def _worker(self) -> None:
        while not self._stop_evt.is_set():
            try:
                msg = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._ser.write((msg + "\n").encode())
                self._ser.flush()
            except Exception as e:
                print(f"[arduino] write failed for '{msg}': {e}")


class NoOpArduinoIO:
    """Drop-in replacement when the Arduino is not connected. All
    pulse/toggle calls are silent no-ops; ping returns False."""

    @property
    def port(self) -> str:
        return "(no arduino)"

    def pulse(self, pin: int) -> None:
        pass

    def toggle(self, pin: int = 13) -> None:
        pass

    def ping(self) -> bool:
        return False

    def close(self) -> None:
        pass


# -----------------------------------------------------------------
#                    Top-level connect() helper
# -----------------------------------------------------------------

def connect(
    port: Optional[str],
    baud: int,
    boot_wait_s: float,
    ping_timeout_s: float,
) -> Optional[ArduinoIO]:
    """Try to open and handshake with the Arduino. Returns an ArduinoIO
    on success, or None if no responsive board was found.

    If `port` is given, only that port is tried (no auto-detect).
    If `port` is None, all available serial ports are tried in order.
    """
    if port is not None:
        ser = _try_handshake(port, baud, boot_wait_s, ping_timeout_s)
        if ser is not None:
            return ArduinoIO(port, ser)
        print(f"[arduino] {port} did not respond to PING")
        return None

    ports = _candidate_ports()
    if not ports:
        print("[arduino] no serial ports found")
        return None

    # First port often boots quickly (~boot_wait_s). To avoid
    # accumulating a long delay scanning many ports, we apply boot_wait
    # only on the first try; subsequent tries assume the device is
    # already past its boot if it was going to respond.
    print(f"[arduino] auto-detecting; trying {len(ports)} port(s) ...")
    first = True
    for p in ports:
        wait = boot_wait_s if first else 0.2
        first = False
        ser = _try_handshake(p, baud, wait, ping_timeout_s)
        if ser is not None:
            return ArduinoIO(p, ser)
    print("[arduino] no port responded to PING")
    return None
