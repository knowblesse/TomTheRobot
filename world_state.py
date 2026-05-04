"""
world_state.py
Shared dataclasses passed between capture / processing / control threads.
All thread-shared mutable state lives in WorldState behind a single lock.
"""

from dataclasses import dataclass, field
from threading import Lock
from typing import Literal


ObjectStatus = Literal["detected", "predicted", "lost"]


@dataclass
class ObjectState:
    """Filtered position + velocity of a tracked object (rat or robot)."""
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    status: ObjectStatus = "lost"
    frames_since_detection: int = 999


@dataclass
class Command:
    """What the controller wants the robot to do this tick."""
    speed: int = 0          # 0–255 in spherov2; we cap at ROBOT_MAX_SPEED
    heading: int = 0        # 0–359, in IMU frame (post-offset corrected)
    stop: bool = True       # True = stop signal regardless of speed/heading


@dataclass
class WorldState:
    """All state shared between threads. Guarded by self.lock."""
    lock: Lock = field(default_factory=Lock)

    # Tracking outputs
    timestamp: float = 0.0
    frame_idx: int = 0
    rat: ObjectState = field(default_factory=ObjectState)
    robot: ObjectState = field(default_factory=ObjectState)
    occlusion: bool = False
    dropped_frame: bool = False

    # Most recent raw detection coordinates (None if not detected this frame).
    # These are pre-Kalman, useful for display so the user sees instant
    # detection feedback rather than smoothed state.
    rat_raw_xy: tuple = (None, None)
    robot_raw_xy: tuple = (None, None)

    # Most recently observed detection area (pixels^2). Updated only when
    # a fresh detection is found; persists across "predicted" frames so
    # the user can see the last known size for tuning area thresholds.
    # Initialized to 0 meaning "no detection yet."
    rat_last_area: float = 0.0
    robot_last_area: float = 0.0

    # Control state
    last_command: Command = field(default_factory=Command)

    # Lifecycle
    running: bool = True
    robot_enabled: bool = True       # space toggles this

    def snapshot(self) -> "WorldStateSnapshot":
        """Take a copy of the read-only fields under lock."""
        with self.lock:
            return WorldStateSnapshot(
                timestamp=self.timestamp,
                frame_idx=self.frame_idx,
                rat=ObjectState(**self.rat.__dict__),
                robot=ObjectState(**self.robot.__dict__),
                rat_raw_xy=self.rat_raw_xy,
                robot_raw_xy=self.robot_raw_xy,
                rat_last_area=self.rat_last_area,
                robot_last_area=self.robot_last_area,
                occlusion=self.occlusion,
                dropped_frame=self.dropped_frame,
                last_command=Command(**self.last_command.__dict__),
                running=self.running,
                robot_enabled=self.robot_enabled,
            )


@dataclass
class WorldStateSnapshot:
    """Lock-free read-only view, taken via WorldState.snapshot()."""
    timestamp: float
    frame_idx: int
    rat: ObjectState
    robot: ObjectState
    rat_raw_xy: tuple
    robot_raw_xy: tuple
    rat_last_area: float
    robot_last_area: float
    occlusion: bool
    dropped_frame: bool
    last_command: Command
    running: bool
    robot_enabled: bool
