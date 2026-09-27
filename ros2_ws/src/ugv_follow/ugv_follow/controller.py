"""Follow controller: keep a fixed gap between the robot's nose and the person. Pure Python, no ROS.

* never reverses — if the person comes closer than the gap, the robot just stops
* a person far to the side (> turn_in_place) → tank turn toward them first
* speed caps stay below the driver's hard limits (and the whole safety chain still applies)
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FollowConfig:
    gap: float = 0.40             # m, nose → person
    nose: float = 0.555           # m, axle → nose
    deadband: float = 0.05        # m around the gap where the robot stays put
    k_v: float = 0.8              # (m/s) per m of gap error
    k_w: float = 1.5              # (rad/s) per rad of bearing
    v_max: float = 0.45
    w_max: float = 0.8
    turn_in_place: float = math.radians(40)


def follow_command(cfg: FollowConfig, x: float, y: float) -> tuple[float, float]:
    """Target at (x, y) in base_footprint (axle frame) → (v, w)."""
    bearing = math.atan2(y, x)
    w = max(-cfg.w_max, min(cfg.w_max, cfg.k_w * bearing))
    if abs(bearing) > cfg.turn_in_place:
        return 0.0, w                                  # face the person first (tank turn)
    gap_now = math.hypot(x, y) - cfg.nose
    err = gap_now - cfg.gap
    if err <= cfg.deadband:
        return 0.0, w if abs(bearing) > math.radians(5) else 0.0
    v = min(cfg.v_max, cfg.k_v * err) * math.cos(bearing) ** 2
    return v, w
