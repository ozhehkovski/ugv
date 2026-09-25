"""Binning raw RPLIDAR measurements into a fixed LaserScan range array. Pure Python, no ROS."""
from __future__ import annotations

import math
from collections.abc import Iterable


def bin_scan(
    measurements: Iterable[tuple[int, float, float]],
    bins: int,
    range_min: float,
    range_max: float,
    invert: bool,
    angle_offset_deg: float = 0.0,
) -> list[float]:
    """(quality, angle_deg, dist_mm) → ranges[bins] (m), CCW from angle 0, `inf` = no return.

    RPLIDAR reports angles clockwise; ROS expects counter-clockwise → invert=True.
    When two points fall into one bin the nearer one wins (conservative for obstacles).
    """
    ranges = [math.inf] * bins
    offset = int(round(angle_offset_deg / 360.0 * bins)) % bins
    for quality, angle, dist in measurements:
        if quality <= 0 or dist <= 0:
            continue
        r = dist / 1000.0
        if not range_min <= r <= range_max:
            continue
        idx = int(round(angle / 360.0 * bins)) % bins
        if invert:
            idx = (bins - idx) % bins
        idx = (idx + offset) % bins
        if r < ranges[idx]:
            ranges[idx] = r
    return ranges
