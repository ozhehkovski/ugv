"""Dead-man gate for browser teleop. Pure Python, no ROS.

The browser re-sends the joystick state every ~100 ms while it is held. If the updates stop
(tab closed, Wi-Fi dropped, phone locked) the gate closes and nothing is published anymore.
"""
from __future__ import annotations

import math


class TeleopGate:
    def __init__(self, timeout: float, max_linear: float, max_angular: float) -> None:
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_angular = max_angular
        self._v = self._w = 0.0
        self._t: float | None = None

    def set(self, v: float, w: float, now: float) -> None:
        if not (math.isfinite(v) and math.isfinite(w)):
            raise ValueError("non-finite teleop command")
        self._v = max(-self.max_linear, min(self.max_linear, v))
        self._w = max(-self.max_angular, min(self.max_angular, w))
        self._t = now

    def cancel(self) -> None:
        self._t = None

    def current(self, now: float) -> tuple[float, float] | None:
        """(v, w) while the operator holds the control, else None."""
        if self._t is None or now - self._t > self.timeout:
            return None
        return self._v, self._w
