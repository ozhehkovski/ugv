"""Direction-aware collision governor. Pure numpy, no ROS.

For a requested (v, w) the robot body (an axis-aligned rectangle in base_footprint, origin at
the drive-axle center) is simulated along the arc for `horizon` seconds. The command is scaled by
how soon the body would come closer than `stop_margin` to any obstacle point:

    scale = time_to_violation / horizon      (1.0 when nothing is violated within the horizon)

A step only counts as a violation when the clearance is below the margin AND shrinking. So moving
away from, or sliding along, an obstacle that is already close is always allowed: the robot can
never be trapped. Nav2 Humble's collision_monitor stop polygon has no notion of direction and does
trap it (any command is zeroed while a point is inside the polygon).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .diff_drive import apply_min_wheel_speed, body_to_wheels

CLOSING_EPS = 0.002  # m: clearance must shrink by more than this to count as "closing in"


@dataclass(frozen=True)
class Body:
    front: float   # x of the nose (m, > 0)
    rear: float    # x of the tail (m, < 0)
    half_width: float

    def clearance(self, pts: np.ndarray) -> np.ndarray:
        """Distance from each point (N,2) in the body frame to the rectangle (0 inside)."""
        dx = np.maximum(np.maximum(self.rear - pts[:, 0], pts[:, 0] - self.front), 0.0)
        dy = np.maximum(np.abs(pts[:, 1]) - self.half_width, 0.0)
        return np.hypot(dx, dy)


@dataclass(frozen=True)
class Decision:
    v: float
    w: float
    scale: float
    reason: str          # "ok" | "slow" | "blocked" | "idle"
    clearance: float     # current minimum clearance, m


class SafetyGovernor:
    def __init__(self, body: Body, *, stop_margin: float = 0.05, horizon: float = 1.5, dt: float = 0.1,
                 track: float = 0.515, max_wheel_speed: float = 0.55, min_wheel_speed: float = 0.0,
                 roi: float = 2.5) -> None:
        self.body = body
        self.stop_margin = stop_margin
        self.horizon = horizon
        self.dt = dt
        self.track = track
        self.max_wheel_speed = max_wheel_speed
        self.min_wheel_speed = min_wheel_speed
        self.roi = roi

    def _relevant(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points.reshape(0, 2)
        c = self.body.clearance(points)
        # points inside the body are self-hits / noise (the lidar sits on top of the body)
        return points[(c > 0.0) & (c < self.roi)]

    def _poses(self, v: float, w: float, horizon: float):
        """Exact unicycle integration: yields (x, y, theta) every dt up to horizon."""
        x = y = th = 0.0
        for _ in range(int(math.ceil(horizon / self.dt))):
            if abs(w) < 1e-6:
                x += v * self.dt * math.cos(th)
                y += v * self.dt * math.sin(th)
            else:
                x += v / w * (math.sin(th + w * self.dt) - math.sin(th))
                y -= v / w * (math.cos(th + w * self.dt) - math.cos(th))
            th += w * self.dt
            yield x, y, th

    def _clearance_at(self, pts: np.ndarray, x: float, y: float, th: float) -> float:
        c, s = math.cos(th), math.sin(th)
        rel = pts - np.array([x, y])
        local = np.column_stack((c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]))
        return float(self.body.clearance(local).min())

    def escape_command(self, points: np.ndarray, candidates: list[tuple[float, float]],
                       duration: float = 1.0) -> tuple[float, float, float] | None:
        """Among short motions the governor allows in full, the one that gains the most clearance.
        Returns (v, w, clearance after `duration`) or None when nothing gains clearance."""
        pts = self._relevant(np.asarray(points, dtype=float).reshape(-1, 2))
        if len(pts) == 0:
            return None
        now = float(self.body.clearance(pts).min())
        best: tuple[float, float, float] | None = None
        for v, w in candidates:
            if self.limit(v, w, pts).reason != "ok":
                continue
            clr = min(self._clearance_at(pts, *p) for p in self._poses(v, w, duration))
            final = self._clearance_at(pts, *list(self._poses(v, w, duration))[-1])
            if clr < min(now, self.stop_margin) - CLOSING_EPS or final <= now + CLOSING_EPS:
                continue
            if best is None or final > best[2]:
                best = (v, w, final)
        return best

    def _time_to_violation(self, v: float, w: float, pts: np.ndarray, horizon: float) -> float | None:
        """First time the body gets closer than stop_margin while closing in; None if never."""
        prev = float(self.body.clearance(pts).min())
        for k, pose in enumerate(self._poses(v, w, horizon), start=1):
            clr = self._clearance_at(pts, *pose)
            if clr < self.stop_margin and clr < prev - CLOSING_EPS:
                return (k - 1) * self.dt
            prev = clr
        return None

    def _effective(self, v: float, w: float) -> tuple[float, float]:
        """The command the driver will really execute (wheel saturation + minimum wheel speed)."""
        left, right = body_to_wheels(v, w, self.track, self.max_wheel_speed)
        left, right = apply_min_wheel_speed(left, right, self.min_wheel_speed)
        return (left + right) / 2.0, (right - left) / self.track

    def limit(self, v: float, w: float, points: np.ndarray) -> Decision:
        pts = self._relevant(np.asarray(points, dtype=float).reshape(-1, 2))
        clearance_now = float(self.body.clearance(pts).min()) if len(pts) else math.inf
        if abs(v) < 1e-4 and abs(w) < 1e-4:
            return Decision(0.0, 0.0, 1.0, "idle", clearance_now)
        if len(pts) == 0:
            return Decision(v, w, 1.0, "ok", clearance_now)
        t = self._time_to_violation(v, w, pts, self.horizon)
        scale = 1.0 if t is None else t / self.horizon
        if scale <= 0.0:
            return Decision(0.0, 0.0, 0.0, "blocked", clearance_now)
        sv, sw = v * scale, w * scale
        # the driver may lift a slow command: make sure the lifted one is still safe for a short while
        ev, ew = self._effective(sv, sw)
        if (abs(ev) > abs(sv) + 1e-6 or abs(ew) > abs(sw) + 1e-6) and \
                self._time_to_violation(ev, ew, pts, 0.5) is not None:
            return Decision(0.0, 0.0, 0.0, "blocked", clearance_now)
        return Decision(sv, sw, scale, "ok" if scale >= 0.999 else "slow", clearance_now)
