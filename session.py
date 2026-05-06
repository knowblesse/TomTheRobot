"""
session.py
Per-session logging: positions CSV + periodic image snapshots.

State machine:
                 1=start              3=halt
    +---------+ ----------> +---------+ ----------> (folder finalized,
    |  IDLE   |             | RUNNING |              new IDLE awaiting 1)
    +---------+ <---------- +---------+
                 ^             |   ^
                 |   2=pause   v   | 1=start (resume)
                 |          +--------+
                 |   3=halt | PAUSED |
                 +--------- +--------+

Public API (called from main.py main loop):
    sm = SessionManager(root_dir, snapshot_every_s=10.0)
    sm.start()       # 1
    sm.pause()       # 2
    sm.halt()        # 3
    sm.log_frame(snap, frame_bgr)   # called every processed frame
    sm.shutdown()    # at program exit

Each call returns a short status string for printing/UI; no exceptions
on no-op transitions (they're explicitly allowed by the table).

Files written per session, under {root}/{timestamp}_session_{nn}/:
    positions.csv     one row per processed frame, including pauses
    snapshots/         periodic JPGs named by frame_idx (e.g. 001234.jpg)
"""

from __future__ import annotations

import csv
import datetime as dt
import threading
from pathlib import Path
from typing import Literal, Optional

import cv2 as cv
import numpy as np

from world_state import WorldStateSnapshot

SessionState = Literal["idle", "running", "paused"]


# CSV schema. Keep stable; older sessions remain readable.
CSV_FIELDS = [
    "timestamp",       # time.monotonic() seconds
    "wall_time",       # ISO 8601 wall-clock time
    "frame_idx",
    "session_id",
    "session_state",   # "running" or "paused"
    "rat_x", "rat_y", "rat_status",
    "robot_x", "robot_y", "robot_status",
    "occlusion",
    "last_cmd_speed", "last_cmd_heading", "last_cmd_stop",
]


class SessionManager:
    """Session state machine + CSV writer + image snapshotter."""

    def __init__(
        self,
        root_dir: str,
        snapshot_every_s: float = 10.0,
        csv_flush_every_n: int = 30,
    ):
        self._root = Path(root_dir)
        self._snapshot_every_s = float(snapshot_every_s)
        self._flush_every_n = int(csv_flush_every_n)

        self._state: SessionState = "idle"
        self._lock = threading.Lock()

        # Set on start(); cleared on halt()
        self._session_dir: Optional[Path] = None
        self._snapshot_dir: Optional[Path] = None
        self._csv_file = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._csv_path: Optional[Path] = None

        # Counters
        self._session_id: Optional[str] = None
        self._row_count = 0
        self._last_snapshot_t: float = 0.0
        self._session_start_t: float = 0.0

    # ----- public properties for the UI -----

    @property
    def state(self) -> SessionState:
        with self._lock:
            return self._state

    @property
    def session_id(self) -> Optional[str]:
        with self._lock:
            return self._session_id

    @property
    def session_dir(self) -> Optional[str]:
        with self._lock:
            return str(self._session_dir) if self._session_dir else None

    @property
    def row_count(self) -> int:
        with self._lock:
            return self._row_count

    # ----- state transitions (1=start, 2=pause, 3=halt) -----

    def start(self) -> str:
        """Key 1. idle->running (new session), paused->running (resume),
        running->no-op."""
        with self._lock:
            if self._state == "idle":
                self._begin_session_locked()
                self._state = "running"
                return f"session started: {self._session_id}"
            if self._state == "paused":
                self._state = "running"
                return f"session resumed: {self._session_id}"
            return "already running (no-op)"

    def pause(self) -> str:
        """Key 2. running->paused, otherwise no-op."""
        with self._lock:
            if self._state == "running":
                self._state = "paused"
                return f"session paused: {self._session_id}"
            return f"cannot pause from {self._state} (no-op)"

    def halt(self) -> str:
        """Key 3. running/paused -> idle (finalize and close); idle no-op."""
        with self._lock:
            if self._state in ("running", "paused"):
                msg = f"session halted: {self._session_id} ({self._row_count} rows)"
                self._end_session_locked()
                self._state = "idle"
                return msg
            return "no session to halt (no-op)"

    # ----- per-frame logging -----

    def log_frame(
        self,
        snap: WorldStateSnapshot,
        frame_bgr: Optional[np.ndarray],
        now: float,
    ) -> None:
        """Append one row to the CSV (if a session exists) and write a
        periodic snapshot image when the interval has elapsed.

        Called from the processing thread once per processed frame. Safe
        no-op if state is idle.
        """
        # Snapshot the locked bits we need quickly
        with self._lock:
            if self._state == "idle" or self._csv_writer is None:
                return
            state_str = self._state
            sid = self._session_id

        cmd = snap.last_command
        row = {
            "timestamp":        f"{now:.6f}",
            "wall_time":        dt.datetime.now().isoformat(timespec="milliseconds"),
            "frame_idx":        snap.frame_idx,
            "session_id":       sid,
            "session_state":    state_str,
            "rat_x":            f"{snap.rat.x:.2f}",
            "rat_y":            f"{snap.rat.y:.2f}",
            "rat_status":       snap.rat.status,
            "robot_x":          f"{snap.robot.x:.2f}",
            "robot_y":          f"{snap.robot.y:.2f}",
            "robot_status":     snap.robot.status,
            "occlusion":        int(snap.occlusion),
            "last_cmd_speed":   cmd.speed,
            "last_cmd_heading": cmd.heading,
            "last_cmd_stop":    int(cmd.stop),
        }

        # Write under lock — DictWriter and file handle are not threadsafe
        with self._lock:
            if self._csv_writer is None:
                return
            self._csv_writer.writerow(row)
            self._row_count += 1
            if self._row_count % self._flush_every_n == 0:
                self._csv_file.flush()

            # Periodic snapshot. Save while holding the lock; cv.imwrite
            # is fast (~5-10ms) and we don't want the file handle to be
            # closed underneath us by a concurrent halt().
            should_snapshot = (
                frame_bgr is not None
                and self._snapshot_dir is not None
                and (now - self._last_snapshot_t) >= self._snapshot_every_s
            )
            if should_snapshot:
                fname = self._snapshot_dir / f"{snap.frame_idx:06d}.jpg"
                self._last_snapshot_t = now
                snapshot_path = str(fname)
                snapshot_img = frame_bgr  # alias, we won't free outside
            else:
                snapshot_path = None
                snapshot_img = None

        # Write the image OUTSIDE the lock so we don't hold it during disk I/O.
        # We already snapped the path under the lock so a concurrent halt()
        # would just leave us writing to a (still-existing) folder.
        if snapshot_path is not None and snapshot_img is not None:
            try:
                cv.imwrite(snapshot_path, snapshot_img,
                           [cv.IMWRITE_JPEG_QUALITY, 85])
            except Exception as e:
                print(f"[session] snapshot save failed: {e}")

    # ----- shutdown -----

    def shutdown(self) -> None:
        """Called from main on exit. Halts an active session cleanly."""
        with self._lock:
            if self._state in ("running", "paused"):
                self._end_session_locked()
                self._state = "idle"

    # ----- internals (must be called with self._lock held) -----

    def _begin_session_locked(self) -> None:
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        # Determine sequence number for this timestamp directory
        n = 1
        while True:
            sid = f"{timestamp}_session_{n:02d}"
            d = self._root / sid
            if not d.exists():
                break
            n += 1
        d.mkdir(parents=True, exist_ok=True)
        snap_dir = d / "snapshots"
        snap_dir.mkdir(exist_ok=True)
        csv_path = d / "positions.csv"

        f = open(csv_path, "w", newline="", encoding="utf-8")
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()

        self._session_dir = d
        self._snapshot_dir = snap_dir
        self._csv_file = f
        self._csv_writer = w
        self._csv_path = csv_path
        self._session_id = sid
        self._row_count = 0
        self._last_snapshot_t = 0.0
        self._session_start_t = 0.0
        print(f"[session] start -> {csv_path}")

    def _end_session_locked(self) -> None:
        try:
            if self._csv_file is not None:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as e:
            print(f"[session] error closing CSV: {e}")
        if self._csv_path is not None:
            print(f"[session] halted -> {self._csv_path} "
                  f"({self._row_count} rows)")
        self._session_dir = None
        self._snapshot_dir = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._session_id = None
