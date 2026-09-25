"""Emergency-stop latch with re-arm. Pure Python, no ROS."""
from __future__ import annotations

ZERO = 1e-3


class EstopLatch:
    """While engaged nothing moves. After release, motion stays blocked until a zero command
    (or silence) is seen, so a joystick still held at release cannot make the robot jump."""

    def __init__(self) -> None:
        self.engaged = False
        self.rearm_needed = False

    def set(self, engaged: bool) -> None:
        if engaged:
            self.engaged = True
            self.rearm_needed = True
        else:
            self.engaged = False

    def allow(self, v: float, w: float, stale: bool) -> bool:
        if self.engaged:
            return False
        if self.rearm_needed:
            if stale or (abs(v) < ZERO and abs(w) < ZERO):
                self.rearm_needed = False
            else:
                return False
        return True

    @property
    def state(self) -> str:
        if self.engaged:
            return "estop"
        return "rearm" if self.rearm_needed else "ok"
