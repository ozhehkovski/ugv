"""Virtual walls: operator-drawn or bump-created segments for obstacles the lidar cannot see
(mirrors, glass). Pure numpy/OpenCV, no ROS.

Stored per map in <maps>/<name>/walls.json as [[x1, y1, x2, y2], ...] in the map frame.
"""
from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence

import cv2
import numpy as np

Segment = tuple[float, float, float, float]
MAX_SEGMENTS = 500
MAX_LENGTH = 20.0      # m


def parse_flat(flat: Sequence[float]) -> list[Segment]:
    if len(flat) % 4:
        raise ValueError("segments must be a multiple of 4 numbers (x1, y1, x2, y2)")
    segs = [tuple(float(v) for v in flat[i:i + 4]) for i in range(0, len(flat), 4)]
    return validate(segs)


def flatten(segs: Sequence[Segment]) -> list[float]:
    return [v for s in segs for v in s]


def validate(segs: Sequence[Segment]) -> list[Segment]:
    if len(segs) > MAX_SEGMENTS:
        raise ValueError(f"too many walls ({len(segs)} > {MAX_SEGMENTS})")
    out: list[Segment] = []
    for s in segs:
        if len(s) != 4 or not all(math.isfinite(v) for v in s):
            raise ValueError(f"invalid segment {s!r}")
        if math.hypot(s[2] - s[0], s[3] - s[1]) > MAX_LENGTH:
            raise ValueError(f"segment longer than {MAX_LENGTH} m")
        out.append((float(s[0]), float(s[1]), float(s[2]), float(s[3])))
    return out


def load(path: str) -> list[Segment]:
    try:
        with open(path, encoding="utf-8") as f:
            return validate([tuple(s) for s in json.load(f)])
    except FileNotFoundError:
        return []


def save(path: str, segs: Sequence[Segment]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump([[round(v, 3) for v in s] for s in segs], f)
    os.replace(tmp, path)


def sample(segs: Sequence[Segment], step: float = 0.025) -> np.ndarray:
    """Points along the segments every `step` metres (N×2), ends included."""
    pts = []
    for x1, y1, x2, y2 in segs:
        n = max(1, int(math.ceil(math.hypot(x2 - x1, y2 - y1) / step)))
        t = np.linspace(0.0, 1.0, n + 1)
        pts.append(np.column_stack((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)))
    return np.vstack(pts) if pts else np.empty((0, 2))


def rasterize(grid: np.ndarray, segs: Sequence[Segment], origin: tuple[float, float], resolution: float,
              value: int = 100) -> np.ndarray:
    """Draw walls into an occupancy grid (rows from the map origin), 1 cell thick, 8-connected."""
    out = grid.copy()
    canvas = np.zeros(grid.shape, dtype=np.uint8)
    ox, oy = origin
    def cell(v: float, o: float) -> int:
        return int(math.floor((v - o) / resolution + 1e-6))     # 0.95 / 0.05 is 18.999… in floats

    for x1, y1, x2, y2 in segs:
        p1 = (cell(x1, ox), cell(y1, oy))
        p2 = (cell(x2, ox), cell(y2, oy))
        cv2.line(canvas, p1, p2, 1, thickness=1, lineType=cv2.LINE_8)
    out[canvas.astype(bool)] = value
    return out


def to_frame(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Map-frame points → the frame of a robot at (x, y, yaw) in the map."""
    c, s = math.cos(yaw), math.sin(yaw)
    rel = points - np.array([x, y])
    return np.column_stack((c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]))


def wall_across(x: float, y: float, yaw: float, contact_x: float, contact_y: float, half_len: float) -> Segment:
    """Map-frame segment perpendicular to the robot heading through a contact point given in the base frame
    (a bump into an invisible obstacle ahead/behind becomes a wall across the path)."""
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = x + c * contact_x - s * contact_y, y + s * contact_x + c * contact_y
    # direction perpendicular to the heading
    dx, dy = -s * half_len, c * half_len
    return (px - dx, py - dy, px + dx, py + dy)
