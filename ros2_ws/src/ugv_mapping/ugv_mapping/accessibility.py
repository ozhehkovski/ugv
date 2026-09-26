"""Accessible-terrain layer from an occupancy grid. numpy + OpenCV, no ROS.

For the real body (0.56 m wide, nose corners sweep 0.62 m around the axle):
  PASSABLE  explored free cell whose distance to any obstacle ≥ half_width + margin
            (the robot fits there when aligned with the passage)
  TURNABLE  distance ≥ turn_radius + margin (a tank turn in place is possible)
Only cells connected to the robot's current cell count (you can actually drive there).
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import cv2
import numpy as np

UNKNOWN, BLOCKED, PASSABLE, TURNABLE = -1, 0, 60, 100
FREE_MAX = 25        # occupancy ≤ this is free (same thresholds as the map image)
OCCUPIED_MIN = 65    # occupancy ≥ this is an obstacle


@dataclass(frozen=True)
class GridInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        """World (x, y) → (row, col) in the grid (row 0 = origin row); None if outside."""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None


def clearance_map(data: Sequence[int], info: GridInfo, gap_fill: float = 0.10) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(known mask, explored-free mask, distance in metres to the nearest OCCUPIED cell).

    A sparse lidar leaves unknown speckles between rays in free space; holes up to `gap_fill`
    are closed so they do not break passages. Large unknown areas stay unexplored (not free).
    Clearance is measured to real obstacles only: an unknown speckle is not a wall.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(info.height, info.width)
    known = grid >= 0
    occupied = grid >= OCCUPIED_MIN
    free = known & (grid <= FREE_MAX)
    k = 2 * max(1, int(round(gap_fill / info.resolution))) + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    explored = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, kernel).astype(bool) & ~occupied
    dist_px = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    return known, explored, dist_px * info.resolution


def accessible_layer(
    data: Sequence[int],
    info: GridInfo,
    robot_xy: tuple[float, float] | None,
    half_width: float,
    turn_radius: float,
    margin: float,
    seed_search: float = 0.5,
) -> np.ndarray:
    """int8 grid (same layout as the input): UNKNOWN / BLOCKED / PASSABLE / TURNABLE."""
    known, explored, dist = clearance_map(data, info)
    passable = explored & (dist >= half_width + margin)
    layer = np.full((info.height, info.width), UNKNOWN, dtype=np.int8)
    layer[known | explored] = BLOCKED
    if robot_xy is None:
        return layer
    seed = _seed_cell(passable, info, robot_xy, seed_search)
    if seed is None:
        return layer
    _n, labels = cv2.connectedComponents(passable.astype(np.uint8), connectivity=8)
    reach = labels == labels[seed]
    layer[reach] = PASSABLE
    layer[reach & (dist >= turn_radius + margin)] = TURNABLE
    return layer


def _seed_cell(passable: np.ndarray, info: GridInfo, xy: tuple[float, float], search: float) -> tuple[int, int] | None:
    """The robot's own cell, or the nearest passable cell within `search` m (robot parked near a wall)."""
    rc = info.cell(*xy)
    if rc is None:
        return None
    if passable[rc]:
        return rc
    r = max(1, int(search / info.resolution))
    r0, c0 = rc
    rows = slice(max(0, r0 - r), min(info.height, r0 + r + 1))
    cols = slice(max(0, c0 - r), min(info.width, c0 + r + 1))
    ys, xs = np.nonzero(passable[rows, cols])
    if len(ys) == 0:
        return None
    ys, xs = ys + rows.start, xs + cols.start
    k = int(np.argmin((ys - r0) ** 2 + (xs - c0) ** 2))
    return int(ys[k]), int(xs[k])
