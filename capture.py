"""
capture.py
Threaded webcam grabber.

Why a thread:
  cv2.VideoCapture.read() blocks until a fresh frame arrives. If the
  processing thread does that, any slow processing iteration causes
  *old* frames to back up in the camera/OS buffer. By the time we read
  the next one, we're tracking the past.

  The fix is to read in a dedicated thread that ALWAYS reads as fast
  as the camera delivers, and overwrite a single "latest" slot. The
  processing thread asks for "the latest frame you have" — never waits
  for one, never gets a stale one.

Backend / format choices (Windows-focused; see config.py):
  - CAP_DSHOW (DirectShow) is the most reliable backend on Windows for
    USB UVC webcams. The default MSMF backend has known issues with
    BUFFERSIZE=1 not being honored.
  - MJPG fourcc lets the camera stream compressed frames over USB 2.0
    at 30 fps for 640x480; uncompressed YUYV often caps at 10–15 fps.
  - BUFFERSIZE=1 instructs the backend to keep at most one frame queued.
"""

import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2 as cv
import numpy as np


@dataclass
class Frame:
    """A single captured frame, with metadata for downstream timing."""
    image: np.ndarray
    timestamp: float       # time.monotonic() at the moment cap.read() returned
    frame_idx: int         # monotonically increasing, set by FrameGrabber


def _select_backend() -> int:
    """Pick the right cv2 backend constant for this OS."""
    if sys.platform.startswith("win"):
        return cv.CAP_DSHOW
    if sys.platform.startswith("linux"):
        return cv.CAP_V4L2
    # macOS / fallback
    return cv.CAP_ANY


class FrameGrabber:
    """Threaded webcam reader with latest-frame semantics."""

    def __init__(
        self,
        camera_index: int,
        width: int,
        height: int,
        fps: int,
        fourcc: str = "MJPG",
    ):
        self._index = camera_index
        self._width = width
        self._height = height
        self._fps = fps
        self._fourcc = fourcc

        self._cap: Optional[cv.VideoCapture] = None
        self._latest: Optional[Frame] = None
        self._latest_lock = threading.Lock()

        self._thread: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()
        self._frame_count = 0      # only incremented in capture thread

        # Stats for debug
        self._last_read_time: float = 0.0
        self._read_intervals = []  # rolling window of recent dt's

    # ----- lifecycle -----

    def open(self) -> None:
        """Open the camera and apply settings. Raises on failure."""
        backend = _select_backend()
        cap = cv.VideoCapture(self._index, backend)
        if not cap.isOpened():
            raise RuntimeError(
                f"Could not open camera index {self._index} with backend {backend}. "
                f"Check that the webcam is plugged in and not in use by another app."
            )

        # Order matters on some backends: set fourcc BEFORE resolution
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*self._fourcc))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv.CAP_PROP_FPS, self._fps)
        cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

        # Verify what we actually got — backends silently override requests
        actual_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv.CAP_PROP_FPS)
        print(f"[capture] Opened camera {self._index}: "
              f"{actual_w}x{actual_h} @ {actual_fps:.1f} fps requested")

        # Warm up: drop the first few frames (some webcams emit junk at start)
        for _ in range(3):
            cap.read()

        self._cap = cap

    def start(self) -> None:
        """Start the background reader thread."""
        if self._cap is None:
            raise RuntimeError("Call open() before start()")
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._run, name="FrameGrabber", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the thread and release the camera."""
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        print("[capture] Stopped.")

    def __enter__(self):
        self.open()
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    # ----- public API for processing thread -----

    def get_latest(self) -> Optional[Frame]:
        """Return the most recently captured frame, or None if none yet.
        Thread-safe; non-blocking. Same frame may be returned twice if the
        consumer is faster than the camera — caller should check frame_idx.
        """
        with self._latest_lock:
            return self._latest

    def measured_fps(self, window: int = 30) -> float:
        """Best-effort instantaneous FPS, computed over the last `window` reads."""
        if not self._read_intervals:
            return 0.0
        recent = self._read_intervals[-window:]
        avg = sum(recent) / len(recent)
        return 1.0 / avg if avg > 0 else 0.0

    # ----- internal capture loop -----

    def _run(self) -> None:
        assert self._cap is not None
        while not self._stop_evt.is_set():
            ok, img = self._cap.read()
            now = time.monotonic()
            if not ok or img is None:
                # transient read failure; brief pause and retry
                time.sleep(0.005)
                continue

            self._frame_count += 1
            frame = Frame(image=img, timestamp=now, frame_idx=self._frame_count)
            with self._latest_lock:
                self._latest = frame

            if self._last_read_time > 0:
                dt = now - self._last_read_time
                self._read_intervals.append(dt)
                # cap rolling window so we don't grow unbounded
                if len(self._read_intervals) > 120:
                    self._read_intervals = self._read_intervals[-60:]
            self._last_read_time = now
