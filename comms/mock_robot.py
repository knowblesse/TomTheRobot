"""
comms/mock_robot.py
A no-hardware robot interface that just logs commands.
Useful for testing the controller and pipeline without a real RVR.
"""

import time
from .interface import RobotInterface


class MockRobot(RobotInterface):

    def __init__(self, verbose: bool = True):
        self._verbose = verbose
        self._connected = False
        self._heading = 0
        self._speed = 0

    def _log(self, msg: str) -> None:
        if self._verbose:
            print(f"[mock t={time.time():.2f}] {msg}")

    def connect(self) -> None:
        self._connected = True
        self._log("connect")

    def disconnect(self) -> None:
        self._connected = False
        self._log("disconnect")

    def set_heading(self, heading_deg: int) -> None:
        self._heading = int(heading_deg) % 360
        self._log(f"set_heading {self._heading}")

    def set_speed(self, speed: int) -> None:
        self._speed = max(-255, min(255, int(speed)))
        self._log(f"set_speed {self._speed}")

    def stop(self) -> None:
        self._speed = 0
        self._log("stop")

    def reset_aim(self) -> None:
        self._heading = 0
        self._log("reset_aim")
