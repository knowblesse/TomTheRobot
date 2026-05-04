"""
tracker.py
Two-object tracking with Kalman filters and occlusion handling.

Design (see design_doc.md §4.4):

  - One KalmanTracker per object (rat, robot). Constant-velocity model.
    State: [x, y, vx, vy]. Measurement: [x, y].

  - Each frame:
      1. predict() both filters (advance state by dt, inflate covariance)
      2. Try to associate each detection with its filter, gating by
         ASSOCIATION_GATE_PX. Reject outliers (probably false detections).
      3. Detect occlusion: predicted positions within OCCLUSION_DISTANCE_PX
         AND fewer than 2 valid detections. If occluded, do not update
         from a single ambiguous detection — predict both filters instead.
      4. Update filters from associated measurements.
      5. Update status (detected / predicted / lost) and emit ObjectStates.

  - Identity is implicit: each filter tracks its own kind (red marker vs.
    dark hood). We don't do Hungarian-style cross-class association
    because the detector already separates them.

  - Re-acquisition after "lost": if the filter is in "lost" state and a
    fresh detection arrives within the arena (no gating against stale
    prediction), we re-initialize the filter at the detection. This
    prevents permanent loss when an object reappears far from where it
    was last seen.

The tracker is stateful but has no threads of its own. The processing
thread calls TwoObjectTracker.update() once per frame. Locking on
WorldState (where results land) is the caller's responsibility.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from detection import Detection
from world_state import ObjectState


# -----------------------------------------------------------------
#                  Single-object Kalman filter
# -----------------------------------------------------------------

class KalmanTracker:
    """Constant-velocity Kalman filter for 2D position tracking.

    Implemented directly with numpy rather than via cv2.KalmanFilter or
    filterpy: small, transparent, no extra dependency. ~30 LOC of math.

    State vector x = [px, py, vx, vy], in pixels and pixels/second.
    Measurement vector z = [px, py].

    Tunable noise:
      process_noise: how much velocity can change per second (Q).
                     Higher = filter follows measurements more closely.
      measurement_noise: how noisy the centroid measurement is (R).
                        Higher = filter trusts predictions more, smooths heavily.
    """

    def __init__(self, process_noise: float, measurement_noise: float):
        self._q = float(process_noise)
        self._r = float(measurement_noise)

        # State and covariance — uninitialized until first measurement
        self._x = np.zeros(4, dtype=np.float64)        # [px, py, vx, vy]
        self._P = np.eye(4, dtype=np.float64) * 1e3    # large initial uncertainty

        # Measurement matrix H (constant): we observe position only
        self._H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0]], dtype=np.float64)

        # Measurement noise covariance R (constant)
        self._R = np.eye(2, dtype=np.float64) * (self._r ** 2)

        self._initialized = False

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def position(self) -> Tuple[float, float]:
        return float(self._x[0]), float(self._x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self._x[2]), float(self._x[3])

    def initialize(self, x: float, y: float) -> None:
        """Seed the filter at a given position with zero velocity and
        moderately tight covariance (we just measured this point)."""
        self._x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        # After a fresh measurement, position uncertainty ~ R, velocity unknown
        P = np.eye(4, dtype=np.float64)
        P[0, 0] = self._r ** 2
        P[1, 1] = self._r ** 2
        P[2, 2] = 1e3
        P[3, 3] = 1e3
        self._P = P
        self._initialized = True

    def predict(self, dt: float) -> Tuple[float, float]:
        """Advance state by dt seconds. Returns the predicted (x, y).
        No-op if the filter has not been initialized yet."""
        if not self._initialized:
            return float(self._x[0]), float(self._x[1])

        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float64)

        # Process noise: random acceleration over dt produces position
        # variance (q*dt^2/2)^2 and velocity variance (q*dt)^2. We use a
        # standard kinematic Q matrix (Bar-Shalom):
        q = self._q
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = q * np.array([[dt4 / 4, 0, dt3 / 2, 0],
                          [0, dt4 / 4, 0, dt3 / 2],
                          [dt3 / 2, 0, dt2, 0],
                          [0, dt3 / 2, 0, dt2]], dtype=np.float64)

        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        return float(self._x[0]), float(self._x[1])

    def update(self, x: float, y: float) -> None:
        """Fold a position measurement into the state estimate.
        Auto-initializes if this is the first measurement."""
        if not self._initialized:
            self.initialize(x, y)
            return

        z = np.array([x, y], dtype=np.float64)
        y_innov = z - (self._H @ self._x)
        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y_innov
        I = np.eye(4, dtype=np.float64)
        self._P = (I - K @ self._H) @ self._P

    def reset(self) -> None:
        """Forget everything. Used when an object is declared lost."""
        self._x = np.zeros(4, dtype=np.float64)
        self._P = np.eye(4, dtype=np.float64) * 1e3
        self._initialized = False


# -----------------------------------------------------------------
#                  Two-object tracker (rat + robot)
# -----------------------------------------------------------------

@dataclass
class TrackerConfig:
    process_noise_rat: float
    process_noise_robot: float
    measurement_noise: float
    max_frames_lost: int
    association_gate_px: float
    occlusion_distance_px: float


class TwoObjectTracker:
    """Manages rat + robot Kalman trackers, occlusion logic, and identity.

    Usage:
        tracker = TwoObjectTracker(cfg)
        for each frame:
            rat_state, robot_state, occluded = tracker.update(
                rat_detection, robot_detection, dt
            )
    """

    def __init__(self, cfg: TrackerConfig):
        self._cfg = cfg
        self._rat = KalmanTracker(cfg.process_noise_rat, cfg.measurement_noise)
        self._robot = KalmanTracker(cfg.process_noise_robot, cfg.measurement_noise)

        # frames_since_detection counters per object
        self._rat_misses = 0
        self._robot_misses = 0

    # ----- public API -----

    def update(
        self,
        rat_det: Optional[Detection],
        robot_det: Optional[Detection],
        dt: float,
    ) -> Tuple[ObjectState, ObjectState, bool]:
        """Advance both filters by dt, fold in detections, return states.

        Returns:
            (rat_state, robot_state, occlusion_flag)
        """
        # 1) Predict both filters (no-op if uninitialized)
        if self._rat.initialized:
            self._rat.predict(dt)
        if self._robot.initialized:
            self._robot.predict(dt)

        # 2) Detect occlusion: both initialized, predicted positions close,
        #    and at most one fresh detection.
        #    When occluded, we don't trust a single detection's identity, so
        #    we pretend we got nothing — the filters just keep predicting.
        occluded = False
        if (self._rat.initialized and self._robot.initialized):
            rx, ry = self._rat.position
            ox, oy = self._robot.position
            d = ((rx - ox) ** 2 + (ry - oy) ** 2) ** 0.5
            n_dets = (1 if rat_det is not None else 0) + \
                     (1 if robot_det is not None else 0)
            if d < self._cfg.occlusion_distance_px and n_dets < 2:
                occluded = True

        # 3) Associate (gate against prediction; reject outliers)
        rat_meas = self._associate(self._rat, rat_det) if not occluded else None
        robot_meas = self._associate(self._robot, robot_det) if not occluded else None

        # 4) Update filters from associated measurements
        if rat_meas is not None:
            self._rat.update(*rat_meas)
            self._rat_misses = 0
        else:
            self._rat_misses += 1

        if robot_meas is not None:
            self._robot.update(*robot_meas)
            self._robot_misses = 0
        else:
            self._robot_misses += 1

        # 5) Reset filters that have been lost too long (so re-acquisition
        #    isn't gated against an ancient prediction). Re-initialization
        #    will happen next time a measurement arrives.
        if self._rat_misses > self._cfg.max_frames_lost:
            self._rat.reset()
        if self._robot_misses > self._cfg.max_frames_lost:
            self._robot.reset()

        return (
            self._make_state(self._rat, self._rat_misses),
            self._make_state(self._robot, self._robot_misses),
            occluded,
        )

    # ----- internals -----

    def _associate(
        self,
        filt: KalmanTracker,
        det: Optional[Detection],
    ) -> Optional[Tuple[float, float]]:
        """Decide whether to accept this detection for this filter.

        - If filter is uninitialized, accept any detection (initial seed).
        - If detection is None, return None.
        - Otherwise, accept only if within ASSOCIATION_GATE_PX of prediction.
        """
        if det is None:
            return None
        if not filt.initialized:
            return (det.x, det.y)

        px, py = filt.position
        dx = det.x - px
        dy = det.y - py
        if (dx * dx + dy * dy) ** 0.5 > self._cfg.association_gate_px:
            return None  # reject as outlier
        return (det.x, det.y)

    def _make_state(self, filt: KalmanTracker, misses: int) -> ObjectState:
        """Build an ObjectState describing this filter's current view."""
        if not filt.initialized:
            return ObjectState(
                x=0.0, y=0.0, vx=0.0, vy=0.0,
                status="lost", frames_since_detection=999,
            )
        x, y = filt.position
        vx, vy = filt.velocity
        if misses == 0:
            status = "detected"
        elif misses <= self._cfg.max_frames_lost:
            status = "predicted"
        else:
            status = "lost"
        return ObjectState(
            x=x, y=y, vx=vx, vy=vy,
            status=status,
            frames_since_detection=misses,
        )
