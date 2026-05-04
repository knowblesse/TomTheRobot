"""
controller/chase.py
Chase controller: aim robot at rat, drive at default speed when far,
hold position (rotate-only) when near. Toggle stays on; only the user
or a hard failure ends a chase session.

Behavior:

  IDLE:    no motion. Toggle key flips us into CHASING.
  CHASING: aim at rat's filtered position. Two sub-states based on
           distance, with hysteresis to prevent oscillation:

             FAR  (entered when distance > RESUME_DISTANCE_PX):
                  drive at ROBOT_DEFAULT_SPEED, update heading

             NEAR (entered when distance < STOP_DISTANCE_PX):
                  speed = 0, keep updating heading. Robot rotates in
                  place to track the rat. Resumes driving only when
                  distance crosses RESUME_DISTANCE_PX going outward.

           Conditions that return us to IDLE:
             - robot is "lost" (tracking failure; needs user attention)
             - stuck detector trips while we are driving (not near)
             - user toggles off

  rat "predicted" (recent dropout, Kalman still tracking):
           Continue last command. Resume normal logic when rat re-acquires.

  rat "lost" (Kalman has fully reset; no info on rat location):
           Stop driving but STAY in CHASING. Wait for rat to re-acquire.
           Driving blind is dangerous in an animal experiment — could
           hit the rat from behind or run off the arena. The robot
           halts and waits silently until the rat is detected again,
           at which point chase resumes automatically.

  Heartbeat: even when nothing changes, we re-issue the current command
  every HEARTBEAT_INTERVAL_S seconds, so a single dropped BLE packet
  doesn't strand the robot moving. Sphero's own command-watchdog will
  also stop the robot if BLE drops entirely.

Heading conventions:
  Camera frame: x is image-right, y is image-down (pixels).
  Robot IMU frame: 0=forward, 90=right, 180=back, 270=left.

For step 6 we assume the user has placed the robot so that its forward
direction is aligned with the camera's image-up. The configured
imu_offset_deg (from config.INITIAL_IMU_OFFSET_DEG) handles the case
where the robot is placed in a different orientation at reset_aim time.
"""

import math
from dataclasses import dataclass
from typing import Literal, Optional

import config
from world_state import Command, ObjectState


# ----- Mode and result -----

ChaseMode = Literal["idle", "chasing"]


@dataclass
class StopReason:
    """Why the controller transitioned out of CHASING. Surfaced
    to the UI / logger so the user knows why the robot stopped."""
    reason: str   # 'rat_lost' | 'stuck' | 'user_toggle' | 'reaim'
    detail: str = ""


# ----- The controller -----

class ChaseController:
    """Chase controller with hysteretic near/far behavior.

    Per-frame contract:
      ctrl.toggle_chase()                  # external toggle key handler
      cmd, reason = ctrl.decide(rat_state, robot_state, t)

    `cmd` is None when the controller decided nothing should be sent
    this tick (deadband + heartbeat tells us no command is needed).
    `reason` is non-None on the tick where we transitioned to IDLE.

    Public sub-state info:
      ctrl.near       True iff in CHASING and within stop distance
                      (rotating in place rather than driving).
    """

    def __init__(self, imu_offset_deg: float = 0.0):
        self._mode: ChaseMode = "idle"
        self._near: bool = False
        self._imu_offset_deg = float(imu_offset_deg)

        # Last command actually issued, for deadband + heartbeat
        self._last_cmd: Optional[Command] = None
        self._last_cmd_time: float = 0.0

        # Stuck detector state
        self._stuck_start: Optional[float] = None

    @property
    def mode(self) -> ChaseMode:
        return self._mode

    @property
    def near(self) -> bool:
        return self._near

    @property
    def imu_offset_deg(self) -> float:
        return self._imu_offset_deg

    # --- external triggers ---

    def toggle_chase(self) -> StopReason | None:
        """Flip mode. Returns a StopReason iff toggling caused an IDLE
        transition; None if we just turned chase on."""
        if self._mode == "chasing":
            self._mode = "idle"
            self._near = False
            self._reset_stuck()
            return StopReason("user_toggle")
        else:
            self._mode = "chasing"
            self._near = False
            self._reset_stuck()
            return None

    def force_stop(self) -> None:
        """Hard stop (e.g. emergency space-bar). Returns to IDLE silently."""
        self._mode = "idle"
        self._near = False
        self._reset_stuck()

    # --- per-frame decision ---

    def decide(
        self,
        rat: ObjectState,
        robot: ObjectState,
        now: float,
    ) -> tuple[Optional[Command], Optional[StopReason]]:
        """Compute the command for this tick.

        Returns (command, stop_reason).
          command is None if no command needs to be sent right now.
          stop_reason is non-None on the tick we transition to IDLE.
        """

        if self._mode == "idle":
            return None, None

        # -------- CHASING --------

        # If we don't even know where the robot is, we can't aim. Stop
        # and exit chase — losing the robot's marker is a tracking
        # failure that needs user attention (move robot, re-sample, etc.).
        if robot.status == "lost":
            self._mode = "idle"
            self._near = False
            return self._make_stop_cmd(now), StopReason(
                "rat_lost", "robot position lost"
            )

        # Rat fully lost: stop driving, but STAY in CHASING. We don't
        # know where the rat is, so blind driving is dangerous (could
        # drive into the rat or off the arena). Issue a stop command if
        # we were last driving, otherwise heartbeat. Resume normal logic
        # automatically when the rat re-acquires.
        if rat.status == "lost":
            self._near = False
            self._reset_stuck()
            # Need to actually halt the robot if we were driving.
            if self._last_cmd is not None and not self._last_cmd.stop:
                return self._make_stop_cmd(now), None
            return self._maybe_heartbeat(now), None

        # Rat predicted but still tracked: continue last cmd. Skip recompute
        # except for an occasional heartbeat.
        if rat.status == "predicted":
            return self._maybe_heartbeat(now), None

        # Rat detected: normal chase logic.

        # --- Distance & hysteresis (decides near/far sub-state) ---
        dx = rat.x - robot.x
        dy = rat.y - robot.y
        dist = math.hypot(dx, dy)

        if self._near:
            # Currently NEAR. Resume driving only after rat is well outside.
            if dist >= config.RESUME_DISTANCE_PX:
                self._near = False
        else:
            # Currently FAR. Stop driving once we close inside the stop band.
            if dist <= config.STOP_DISTANCE_PX:
                self._near = True

        # --- Compute desired heading (always; we steer in both states) ---
        # Image y axis points DOWN. atan2(dx, -dy) gives the angle in
        # CAMERA frame from robot to rat: 0=up, 90=right, 180=down, 270=left.
        target_cam_deg = math.degrees(math.atan2(dx, -dy))
        # Convert to IMU frame. imu_offset_deg is the camera-frame angle
        # that IMU heading 0 currently points at (set at reset_aim time).
        # To physically face camera angle θ_cam, we need IMU heading
        # θ_cam - imu_offset_deg (mod 360).
        target_imu_deg = (target_cam_deg - self._imu_offset_deg) % 360.0
        target_heading = int(round(target_imu_deg)) % 360
        target_speed = 0 if self._near else min(
            config.ROBOT_DEFAULT_SPEED, config.ROBOT_MAX_SPEED
        )

        # --- Stuck detection (only meaningful while driving) ---
        if not self._near and self._last_cmd is not None and not self._last_cmd.stop:
            obs_speed = math.hypot(robot.vx, robot.vy)
            if obs_speed < config.STUCK_VELOCITY_PX_S:
                if self._stuck_start is None:
                    self._stuck_start = now
                elif now - self._stuck_start >= config.STUCK_DURATION_S:
                    self._mode = "idle"
                    self._near = False
                    self._reset_stuck()
                    return self._make_stop_cmd(now), StopReason(
                        "stuck",
                        f"obs_speed={obs_speed:.1f}px/s for "
                        f"{config.STUCK_DURATION_S:.1f}s"
                    )
            else:
                self._stuck_start = None
        else:
            # Reset stuck timer whenever we are stationary-by-design or had
            # a stop in the queue; only sustained driving without observed
            # motion counts.
            self._stuck_start = None

        # --- Deadband: skip re-issuing identical command ---
        if self._last_cmd is not None and not self._last_cmd.stop:
            d_heading = _heading_diff(target_heading, self._last_cmd.heading)
            if (d_heading < config.HEADING_DEADBAND_DEG
                    and target_speed == self._last_cmd.speed):
                return self._maybe_heartbeat(now), None

        # --- Issue the command ---
        cmd = Command(speed=target_speed, heading=target_heading, stop=False)
        self._last_cmd = cmd
        self._last_cmd_time = now
        return cmd, None

    # --- helpers ---

    def _make_stop_cmd(self, now: float) -> Command:
        cmd = Command(speed=0, heading=0, stop=True)
        self._last_cmd = cmd
        self._last_cmd_time = now
        return cmd

    def _maybe_heartbeat(self, now: float) -> Optional[Command]:
        """Re-issue the current command if it's been longer than the
        heartbeat interval, otherwise return None."""
        if self._last_cmd is None:
            return None
        if now - self._last_cmd_time >= config.HEARTBEAT_INTERVAL_S:
            self._last_cmd_time = now
            return self._last_cmd
        return None

    def _reset_stuck(self) -> None:
        self._stuck_start = None


def _heading_diff(a: int, b: int) -> float:
    """Smallest signed angular difference between two headings (0-359)."""
    d = (a - b + 540) % 360 - 180
    return abs(d)
