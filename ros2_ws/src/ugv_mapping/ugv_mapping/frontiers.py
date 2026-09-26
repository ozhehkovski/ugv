"""Frontier detection on the accessible-terrain layer. numpy + OpenCV, no ROS.

A frontier cell is a cell the robot can reach (PASSABLE/TURNABLE) close to UNEXPLORED space.
Frontier cells are clustered; each cluster gets a goal cell inside it (reachable by construction)
and a heading that looks into the unknown.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .accessibility import PASSABLE, TURNABLE, UNKNOWN, GridInfo


@dataclass(frozen=True)
class Frontier:
    x: float
    y: float
    yaw: float          # look toward the unexplored side
    size: int           # cells in the cluster
    distance: float     # straight-line distance from the robot, m


def find_frontiers(layer: np.ndarray, info: GridInfo, robot_xy: tuple[float, float],
                   reach: float = 0.25, min_size_m: float = 0.3, min_unknown_m2: float = 0.1,
                   goal_search: float = 0.6) -> list[Frontier]:
    """layer: int8 grid (rows from the map origin) as produced by accessible_layer().

    The goal of each cluster is the TURNABLE cell nearest to its centre (within goal_search):
    the robot must be able to tank-turn where it stops, otherwise it can drive into a dead end
    it cannot leave (no reverse planning, no rear camera). Clusters without one are skipped."""
    reachable = (layer == PASSABLE) | (layer == TURNABLE)
    turnable = layer == TURNABLE
    unknown = _large_regions((layer == UNKNOWN).astype(np.uint8), int(min_unknown_m2 / info.resolution ** 2))
    k = 2 * max(1, int(round(reach / info.resolution))) + 1
    near_unknown = cv2.dilate(unknown, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))).astype(bool)
    frontier = (reachable & near_unknown).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(frontier, connectivity=8)
    min_cells = max(1, int(min_size_m / info.resolution))
    out: list[Frontier] = []
    for i in range(1, n):
        size = int(stats[i, cv2.CC_STAT_AREA])
        if size < min_cells:
            continue
        rows, cols = np.nonzero(labels == i)
        cell = _turnable_goal(turnable, rows.mean(), cols.mean(), int(goal_search / info.resolution))
        if cell is None:
            continue            # e.g. a dead-end corridor: the robot could get in but not turn around
        r, c = cell
        x = info.origin_x + (c + 0.5) * info.resolution
        y = info.origin_y + (r + 0.5) * info.resolution
        out.append(Frontier(x, y, _look_yaw(unknown, r, c, info, robot_xy), size,
                            math.hypot(x - robot_xy[0], y - robot_xy[1])))
    return out


def _turnable_goal(turnable: np.ndarray, cr: float, cc: float, radius: int) -> tuple[int, int] | None:
    r0, c0 = int(round(cr)), int(round(cc))
    rows = slice(max(0, r0 - radius), r0 + radius + 1)
    cols = slice(max(0, c0 - radius), c0 + radius + 1)
    ys, xs = np.nonzero(turnable[rows, cols])
    if len(ys) == 0:
        return None
    ys, xs = ys + rows.start, xs + cols.start
    d2 = (ys - cr) ** 2 + (xs - cc) ** 2
    k = int(np.argmin(d2))
    if d2[k] > radius ** 2:
        return None
    return int(ys[k]), int(xs[k])


def _large_regions(mask: np.ndarray, min_cells: int) -> np.ndarray:
    """Drop unknown specks (single cells between lidar rays): they are not worth a trip."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = np.zeros(n, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_cells
    return keep[labels].astype(np.uint8)


def _look_yaw(unknown: np.ndarray, r: int, c: int, info: GridInfo, robot_xy: tuple[float, float]) -> float:
    """Heading from the goal toward the centroid of nearby unknown cells (else away from the robot)."""
    w = max(2, int(1.0 / info.resolution))
    r0, c0 = max(0, r - w), max(0, c - w)
    ur, uc = np.nonzero(unknown[r0:r + w + 1, c0:c + w + 1])
    if len(ur):
        return math.atan2((ur.mean() + r0) - r, (uc.mean() + c0) - c)
    x = info.origin_x + (c + 0.5) * info.resolution
    y = info.origin_y + (r + 0.5) * info.resolution
    return math.atan2(y - robot_xy[1], x - robot_xy[0])


def choose_frontier(frontiers: list[Frontier], blacklist: list[tuple[float, float]],
                    blacklist_radius: float = 0.5, min_distance: float = 0.4,
                    size_weight: float = 0.02, distance_weight: float = 1.0) -> Frontier | None:
    """Best frontier: large and close; skip failed spots and the one the robot is standing on."""
    best, best_score = None, -math.inf
    for f in frontiers:
        if f.distance < min_distance:
            continue
        if any(math.hypot(f.x - bx, f.y - by) < blacklist_radius for bx, by in blacklist):
            continue
        score = size_weight * f.size - distance_weight * f.distance
        if score > best_score:
            best, best_score = f, score
    return best
