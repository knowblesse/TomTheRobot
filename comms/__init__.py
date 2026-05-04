"""comms package: robot communication layer."""
from .interface import RobotInterface
from .mock_robot import MockRobot

__all__ = ["RobotInterface", "MockRobot"]
