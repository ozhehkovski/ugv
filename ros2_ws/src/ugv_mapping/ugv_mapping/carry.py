"""Is the robot being carried? IMU tilt / vertical acceleration while nobody commands the wheels.
Pure Python, no ROS.

Carrying the robot moves it without wheel odometry: SLAM then glues the new view of the room at a wrong
place (rooms "behind walls"). Detecting it lets SLAM pause and the operator confirm the new pose.
Driving over a threshold also tilts the robot, but then the wheels are commanded — not a carry.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CarryConfig:
    tilt: float = 0.12            # rad (~7°) away from the resting attitude
    accel_dev: float = 2.5        # m/s² vertical acceleration away from gravity
    quiet_time: float = 1.0       # s without wheel commands before an anomaly counts as a carry
    on_time: float = 0.3          # s of anomaly → carried
    off_time: float = 2.0         # s calm → put down


class CarryDetector:
    def __init__(self, cfg: CarryConfig | None = None) -> None:
        self.cfg = cfg or CarryConfig()
        self.carried = False
        self._anomaly_since: float | None = None
        self._calm_since: float | None = None
        self._last_cmd = -1e9

    def command(self, now: float) -> None:
        """Call whenever the wheels are commanded to move."""
        self._last_cmd = now

    def update(self, now: float, tilt: float, accel_dev: float) -> bool:
        c = self.cfg
        driving = now - self._last_cmd < c.quiet_time
        anomaly = not driving and (abs(tilt) > c.tilt or abs(accel_dev) > c.accel_dev)
        if anomaly:
            self._calm_since = None
            if self._anomaly_since is None:
                self._anomaly_since = now
            if not self.carried and now - self._anomaly_since >= c.on_time:
                self.carried = True
        else:
            self._anomaly_since = None
            if self._calm_since is None:
                self._calm_since = now
            if self.carried and now - self._calm_since >= c.off_time:
                self.carried = False
        return self.carried
