"""
comms/interface.py
Abstract robot interface. The rest of the codebase talks to this only,
so we can swap spherov2 for another library, a mock, or a different robot
without touching detection, tracking, or controller code.
"""

from abc import ABC, abstractmethod


class RobotInterface(ABC):

    @abstractmethod
    def connect(self) -> None:
        """Establish connection. Blocks until connected or raises."""

    @abstractmethod
    def disconnect(self) -> None:
        """Close connection cleanly."""

    @abstractmethod
    def set_heading(self, heading_deg: int) -> None:
        """Set desired heading in IMU frame. 0–359."""

    @abstractmethod
    def set_speed(self, speed: int) -> None:
        """Set forward speed. 0–255 (spherov2 convention)."""

    @abstractmethod
    def stop(self) -> None:
        """Stop motion immediately."""

    @abstractmethod
    def reset_aim(self) -> None:
        """Reset IMU heading reference to current orientation = 0°."""
