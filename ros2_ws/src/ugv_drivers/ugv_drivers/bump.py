"""Bump detection from the drive itself: catches obstacles the lidar cannot see (mirrors, glass).
Pure Python, no ROS.

Stall rule: a wheel is commanded to move but turns much slower than commanded while its motor current is
high — the robot is pushing against something. Impact rule (optional, off by default until its threshold
is measured on the real floor): a sharp deceleration against the direction of travel.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BumpConfig:
    min_target: float = 0.05       # m/s: ignore crawling commands
    stall_ratio: float = 0.3       # measured < ratio × commanded ...
    stall_current: float = 3.0     # ... with |motor current| above this (A) ...
    stall_time: float = 0.4        # ... for this long (s)
    impact_accel: float = 0.0      # m/s² deceleration against the motion; 0 = disabled
    cooldown: float = 2.0          # s between events


class BumpDetector:
    def __init__(self, cfg: BumpConfig) -> None:
        self.cfg = cfg
        self._stall_since: list[float | None] = [None, None]
        self._last_event = -math.inf

    def reset(self) -> None:
        self._stall_since = [None, None]

    def update(self, now: float, targets: tuple[float, float], measured: tuple[float, float],
               currents: tuple[float, float], accel_x: float | None = None) -> str | None:
        """Returns "stall_left" / "stall_right" / "impact" when a bump is detected, else None."""
        c = self.cfg
        event = None
        for i, (tgt, meas, cur) in enumerate(zip(targets, measured, currents)):
            stalled = (abs(tgt) > c.min_target and abs(meas) < c.stall_ratio * abs(tgt)
                       and abs(cur) > c.stall_current)
            if not stalled:
                self._stall_since[i] = None
                continue
            if self._stall_since[i] is None:
                self._stall_since[i] = now
            elif now - self._stall_since[i] >= c.stall_time:
                event = event or ("stall_left" if i == 0 else "stall_right")
        v = (targets[0] + targets[1]) / 2.0
        if (event is None and c.impact_accel > 0.0 and accel_x is not None and abs(v) > c.min_target
                and -math.copysign(1.0, v) * accel_x > c.impact_accel):
            event = "impact"
        if event is None or now - self._last_event < c.cooldown:
            return None
        self._last_event = now
        self.reset()
        return event


def contact_point(left: float, right: float, track: float, front: float, rear: float,
                  half_width: float) -> tuple[float, float]:
    """Most likely contact point (base frame) for the motion being commanded when the bump happened."""
    v = (left + right) / 2.0
    w = (right - left) / track
    if abs(v) >= 0.3 * abs(w) * front:          # mostly driving: the leading edge
        return (front, 0.0) if v > 0 else (rear, 0.0)
    return (front, half_width if w > 0 else -half_width)   # turning in place: the swinging nose corner
