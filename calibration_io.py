"""
calibration_io.py
Save / load session-specific calibration data.

What lives here (persisted across sessions):
  - The arena polygon
  - The sampled RED MARKER LAB color
  - The sampled RAT V threshold

What does NOT live here (defined in config.py):
  - color_range, min/max area thresholds for both objects
  - all tracking, control, and timing parameters

The split is deliberate: numeric tuning knobs live in code (config.py)
because they're version-controlled defaults you change deliberately;
session-specific samples (polygon, color samples) live in JSON because
they're measured values that change every time the rig moves or the
lighting changes.

Format: JSON file at the project root (./last_calibration.json).

Schema (version 3):
{
    "version": 3,
    "frame_size": [height, width],
    "arena_polygon": [[x0, y0], [x1, y1], ...],   # may be empty
    "red_target_lab": [L, a, b]   or null,
    "rat_v_threshold": int        or null
}

Any of the three measured fields can be null/empty; main.py will only
prompt for the missing ones.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

SCHEMA_VERSION = 3


@dataclass
class SessionCalibration:
    """Container for everything persisted across sessions."""
    frame_size: Tuple[int, int]                                 # (h, w)
    arena_polygon: List[Tuple[int, int]] = field(default_factory=list)
    red_target_lab: Optional[Tuple[int, int, int]] = None       # ints 0-255
    rat_v_threshold: Optional[int] = None


def save_calibration(path: str, calib: SessionCalibration) -> None:
    """Write calibration to JSON. Overwrites if it exists."""
    payload = {
        "version": SCHEMA_VERSION,
        "frame_size": [int(calib.frame_size[0]), int(calib.frame_size[1])],
        "arena_polygon": [[int(x), int(y)] for (x, y) in calib.arena_polygon],
        "red_target_lab": (
            [int(c) for c in calib.red_target_lab]
            if calib.red_target_lab is not None else None
        ),
        "rat_v_threshold": (
            int(calib.rat_v_threshold)
            if calib.rat_v_threshold is not None else None
        ),
    }
    Path(path).write_text(json.dumps(payload, indent=2))
    print(f"[calib] saved to {path}")


def load_calibration(path: str) -> Optional[SessionCalibration]:
    """Load calibration from JSON. Returns None if missing or unreadable.
    Raises ValueError on schema version mismatch."""
    p = Path(path)
    if not p.exists():
        return None
    try:
        payload = json.loads(p.read_text())
    except Exception as e:
        print(f"[calib] could not parse {path}: {e}")
        return None

    ver = payload.get("version")
    if ver != SCHEMA_VERSION:
        raise ValueError(
            f"Calibration file {path} has version {ver}, "
            f"expected {SCHEMA_VERSION}. Delete the file to start fresh."
        )

    fs = payload["frame_size"]
    polygon = [tuple(pt) for pt in payload.get("arena_polygon", [])]
    red = payload.get("red_target_lab")
    rat_thr = payload.get("rat_v_threshold")

    print(f"[calib] loaded from {path}  "
          f"(polygon={len(polygon)}pts, "
          f"red={'set' if red else 'unset'}, "
          f"rat={'set' if rat_thr is not None else 'unset'})")
    return SessionCalibration(
        frame_size=(int(fs[0]), int(fs[1])),
        arena_polygon=polygon,
        red_target_lab=tuple(red) if red is not None else None,
        rat_v_threshold=int(rat_thr) if rat_thr is not None else None,
    )
