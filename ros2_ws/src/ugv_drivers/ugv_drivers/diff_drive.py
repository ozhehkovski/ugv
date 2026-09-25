"""Differential-drive kinematics, hard limits and odometry. Pure Python, no ROS."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveLimits:
    max_linear: float        # m/s
    max_angular: float       # rad/s
    max_wheel_speed: float   # m/s
    linear_accel: float      # m/s²
    linear_decel: float      # m/s²
    angular_accel: float     # rad/s²

    @classmethod
    def from_config(cls, limits: dict) -> DriveLimits:
        return cls(
            max_linear=float(limits["max_linear"]),
            max_angular=float(limits["max_angular"]),
            max_wheel_speed=float(limits["max_wheel_speed"]),
            linear_accel=float(limits["linear_accel"]),
            linear_decel=float(limits["linear_decel"]),
            angular_accel=float(limits["angular_accel"]),
        )


def clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


def ramp(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    """Move current toward target. Growing |speed| is limited by accel, shrinking by decel."""
    speeding_up = target * current >= 0.0 and abs(target) > abs(current)
    step = (accel if speeding_up else decel) * dt
    d = target - current
    if abs(d) <= step:
        return target
    return current + math.copysign(step, d)


def body_to_wheels(v: float, w: float, track: float, max_wheel_speed: float) -> tuple[float, float]:
    """(v, w) → (left, right) wheel speeds. If a wheel saturates both are scaled together,
    so the path curvature is preserved (a tank turn stays a turn in place)."""
    left = v - w * track / 2.0
    right = v + w * track / 2.0
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed > 0.0:
        k = max_wheel_speed / peak
        left, right = left * k, right * k
    return left, right


class VelocityLimiter:
    """Clamps and ramps body velocity. Keeps state between calls (one per drive)."""

    def __init__(self, limits: DriveLimits) -> None:
        self.lim = limits
        self.v = 0.0
        self.w = 0.0

    def reset(self) -> None:
        self.v = self.w = 0.0

    def step(self, v_target: float, w_target: float, dt: float) -> tuple[float, float]:
        v_t = clamp(v_target, self.lim.max_linear)
        w_t = clamp(w_target, self.lim.max_angular)
        self.v = ramp(self.v, v_t, self.lim.linear_accel, self.lim.linear_decel, dt)
        self.w = ramp(self.w, w_t, self.lim.angular_accel, self.lim.angular_accel, dt)
        return self.v, self.w


def erpm_per_mps(wheel_radius: float, pole_pairs: int, gear_ratio: float = 1.0) -> float:
    return 60.0 * pole_pairs * gear_ratio / (2.0 * math.pi * wheel_radius)


def meters_per_tach(wheel_radius: float, pole_pairs: int, gear_ratio: float = 1.0) -> float:
    """VESC tachometer: 6 counts per electrical revolution."""
    return 2.0 * math.pi * wheel_radius / (6.0 * pole_pairs * gear_ratio)


class Odometry2D:
    """Integrates wheel travel (midpoint rule)."""

    def __init__(self, track: float) -> None:
        self.track = track
        self.x = self.y = self.theta = 0.0

    def update(self, d_left: float, d_right: float) -> None:
        dc = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.track
        mid = self.theta + dth / 2.0
        self.x += dc * math.cos(mid)
        self.y += dc * math.sin(mid)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
