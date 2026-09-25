"""Yaw-rate PI on the gyro: keeps the heading on straight runs despite the swinging front casters.

The commanded angular velocity is treated as a yaw-rate TARGET. The integral of the rate error is
exactly the accumulated heading error, so with w_target = 0 the loop steers back to the heading
the run started with; in turns it makes the turn rate match the command. Pure Python, no ROS.
"""
from __future__ import annotations


class HeadingHold:
    def __init__(self, k_rate: float, k_heading: float, max_correction: float, max_heading_error: float) -> None:
        self.k_rate = k_rate
        self.k_heading = k_heading
        self.max_correction = max_correction
        self.max_heading_error = max_heading_error
        self.heading_error = 0.0

    def reset(self) -> None:
        self.heading_error = 0.0

    def update(self, w_target: float, w_measured: float, dt: float, active: bool) -> float:
        """Angular-velocity correction (rad/s) to add to w_target. active=False (stopped, no IMU) → 0."""
        if not active:
            self.reset()
            return 0.0
        rate_error = w_target - w_measured
        lim = self.max_heading_error
        self.heading_error = max(-lim, min(lim, self.heading_error + rate_error * dt))
        corr = self.k_rate * rate_error + self.k_heading * self.heading_error
        return max(-self.max_correction, min(self.max_correction, corr))
