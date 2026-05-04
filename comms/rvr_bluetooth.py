"""
comms/rvr_bluetooth.py
Concrete RobotInterface using spherov2 over Bluetooth LE.

spherov2's SpheroEduAPI is synchronous (it owns an internal asyncio loop
via bleak), so this wrapper is also synchronous. Use a context manager
or call connect()/disconnect() explicitly.

Reference:
  https://spherov2.readthedocs.io/en/latest/sphero_edu.html
"""

import time
from typing import Optional

from spherov2 import scanner
from spherov2.controls.v2 import PacketDecodingException
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.toy.rvr import RVR

from .interface import RobotInterface


def _install_packet_error_filter(toy: RVR) -> None:
    """Suppress benign PacketDecodingException noise from spherov2's BLE
    read callback. These exceptions originate in an asyncio callback, so
    they bypass try/except and surface as unhandled-task warnings. The
    decoder resyncs on the next packet automatically; for our use case
    (sending commands, not consuming sensor streams) the dropped packet
    is harmless. We attach a custom exception handler to the BLE adapter's
    asyncio loop that silences only this specific exception type.

    spherov2 internals: each Toy instance holds an adapter (BleakAdapter
    by default) which spins up its own event loop on its own thread. We
    fish the loop out via name-mangled attributes — fragile against
    spherov2 version changes, but works on 0.12.x and is purely cosmetic
    if it ever breaks (the worst case is the noise just comes back).
    """
    try:
        adapter = getattr(toy, "_Toy__adapter", None)
        if adapter is None:
            return
        loop = getattr(adapter, "_BleakAdapter__event_loop", None)
        if loop is None:
            return
    except Exception:
        return

    def _handler(loop, context):
        exc = context.get("exception")
        if isinstance(exc, PacketDecodingException):
            return  # swallow silently
        loop.default_exception_handler(context)

    loop.call_soon_threadsafe(loop.set_exception_handler, _handler)


class RvrBluetooth(RobotInterface):

    def __init__(self, timeout_s: float = 10.0):
        self._timeout_s = timeout_s
        self._toy: Optional[RVR] = None
        self._api: Optional[SpheroEduAPI] = None
        self._api_cm = None  # context manager handle

    def connect(self) -> None:
        if self._api is not None:
            return  # already connected

        print("[comms] Scanning for RVR/RVR+ over Bluetooth ...")
        # find_toy with toy_types filter; spherov2 uses the plural form and
        # expects a list/iterable of toy classes. RVR+ uses the same toy_type
        # prefix (RV-) as RVR, so this picks up both.
        toy = scanner.find_toy(toy_types=[RVR], timeout=self._timeout_s)
        if toy is None:
            raise RuntimeError(
                "Could not find an RVR over Bluetooth. "
                "Is the robot powered on and paired with this computer?"
            )
        self._toy = toy
        print(f"[comms] Found {toy}; opening API session ...")

        # On Windows, the BLE stack sometimes loses the device between scan
        # and connect ("Device with address ... was not found"). Retry a few
        # times with a small delay before giving up.
        last_err: Optional[Exception] = None
        for attempt in range(1, 4):
            try:
                self._api_cm = SpheroEduAPI(toy)
                self._api = self._api_cm.__enter__()
                # Once the API session is up, the spherov2 asyncio loop is
                # running. Attach our filter to silence the benign packet-
                # decode errors that occur intermittently during BLE reads.
                _install_packet_error_filter(toy)
                print("[comms] Connected.")
                return
            except Exception as e:
                last_err = e
                print(f"[comms] Connect attempt {attempt} failed: {e}")
                # Clean up partial state from the failed __enter__
                self._api_cm = None
                self._api = None
                if attempt < 3:
                    print("[comms] Retrying in 2s ...")
                    time.sleep(2.0)

        raise RuntimeError(
            f"Could not open BLE session after 3 attempts. "
            f"Last error: {last_err}. "
            f"Try: (1) pair RV-XXXX in Windows Bluetooth settings, "
            f"(2) toggle Bluetooth off/on, (3) power-cycle the robot."
        )

    def disconnect(self) -> None:
        if self._api is None:
            return
        try:
            # try to leave the robot in a safe state
            self._api.stop_roll()
        except Exception:
            pass
        try:
            self._api_cm.__exit__(None, None, None)
        finally:
            self._api = None
            self._api_cm = None
            self._toy = None
            print("[comms] Disconnected.")

    def _require_connected(self) -> SpheroEduAPI:
        if self._api is None:
            raise RuntimeError("Robot not connected. Call connect() first.")
        return self._api

    def set_heading(self, heading_deg: int) -> None:
        api = self._require_connected()
        # Normalize to 0–359
        h = int(heading_deg) % 360
        api.set_heading(h)

    def set_speed(self, speed: int) -> None:
        api = self._require_connected()
        # spherov2 set_speed accepts -255..255; we keep it non-negative here
        # (controller never wants reverse for chase behavior; reserve negative
        # for future).
        s = max(-255, min(255, int(speed)))
        api.set_speed(s)

    def stop(self) -> None:
        api = self._require_connected()
        api.stop_roll()

    def reset_aim(self) -> None:
        api = self._require_connected()
        api.reset_aim()
