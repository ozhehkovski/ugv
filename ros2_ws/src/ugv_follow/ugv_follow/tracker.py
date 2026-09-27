"""Single-target person tracker in the odom frame. Pure Python/numpy, no ROS.

Lock: the camera-confirmed person closest to the robot, in front (±40°) within 3 m.
Track: alpha-beta filter on (x, y); camera candidates first, lidar leg candidates keep the track alive
when the camera sees only legs (the camera sits 11 cm above the floor) or loses the box for a moment.
Lost: re-acquire only a camera candidate near the last position whose clothes look alike.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .detection import similarity


@dataclass
class Candidate:
    x: float                      # odom frame, m
    y: float
    camera: bool                  # confirmed by the person detector (else: lidar legs only)
    hist: np.ndarray | None = None


@dataclass
class TrackerConfig:
    gate: float = 0.6             # m: association radius around the prediction
    lidar_gate: float = 0.4       # m: tighter for lidar-only candidates (furniture legs look like legs too)
    lost_after: float = 1.0       # s without any association → lost
    lock_range: float = 3.0
    lock_bearing: float = math.radians(40)
    reacquire_radius: float = 2.0
    reacquire_similarity: float = 0.5
    alpha: float = 0.6
    beta: float = 0.2
    max_speed: float = 1.8        # m/s, a walking person


@dataclass
class Target:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    t: float = 0.0
    hist: np.ndarray | None = None
    seen_t: float = 0.0
    history: list = field(default_factory=list)


class Tracker:
    def __init__(self, cfg: TrackerConfig | None = None) -> None:
        self.cfg = cfg or TrackerConfig()
        self.target: Target | None = None

    @property
    def locked(self) -> bool:
        return self.target is not None

    def reset(self) -> None:
        self.target = None

    def lost(self, now: float) -> bool:
        return self.target is not None and now - self.target.seen_t > self.cfg.lost_after

    def try_lock(self, cands: list[Candidate], robot: tuple[float, float, float], now: float) -> bool:
        rx, ry, ryaw = robot
        best, best_d = None, math.inf
        for c in cands:
            if not c.camera:
                continue
            d = math.hypot(c.x - rx, c.y - ry)
            bearing = math.atan2(c.y - ry, c.x - rx) - ryaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if d <= self.cfg.lock_range and abs(bearing) <= self.cfg.lock_bearing and d < best_d:
                best, best_d = c, d
        if best is None:
            return False
        self.target = Target(best.x, best.y, t=now, hist=best.hist, seen_t=now)
        return True

    def update(self, cands: list[Candidate], now: float) -> Candidate | None:
        """Associate one candidate with the target and filter. Returns the used candidate or None."""
        tg = self.target
        if tg is None:
            return None
        dt = max(1e-3, now - tg.t)
        px, py = tg.x + tg.vx * dt, tg.y + tg.vy * dt
        chosen = None
        if not self.lost(now):
            chosen = self._nearest([c for c in cands if c.camera], px, py, self.cfg.gate, tg.hist) \
                or self._nearest([c for c in cands if not c.camera], px, py, self.cfg.lidar_gate, None)
        else:
            chosen = self._reacquire(cands, tg)
        if chosen is None:
            tg.x, tg.y, tg.t = px, py, now          # coast on the prediction
            if self.lost(now):
                tg.vx = tg.vy = 0.0                 # do not coast away forever
            return None
        was_lost = self.lost(now)
        rx, ry = chosen.x - px, chosen.y - py
        a = 1.0 if was_lost else self.cfg.alpha
        tg.x, tg.y = px + a * rx, py + a * ry
        if was_lost:
            tg.vx = tg.vy = 0.0
        else:
            tg.vx += self.cfg.beta * rx / dt
            tg.vy += self.cfg.beta * ry / dt
            sp = math.hypot(tg.vx, tg.vy)
            if sp > self.cfg.max_speed:
                tg.vx, tg.vy = tg.vx * self.cfg.max_speed / sp, tg.vy * self.cfg.max_speed / sp
        tg.t = tg.seen_t = now
        if chosen.camera and chosen.hist is not None:
            tg.hist = chosen.hist if tg.hist is None else (0.9 * tg.hist + 0.1 * chosen.hist)
        return chosen

    @staticmethod
    def _nearest(cands: list[Candidate], x: float, y: float, gate: float, hist: np.ndarray | None) -> Candidate | None:
        best, best_cost = None, math.inf
        for c in cands:
            d = math.hypot(c.x - x, c.y - y)
            if d > gate:
                continue
            cost = d - 0.3 * max(0.0, similarity(hist, c.hist))    # clothes break ties between close people
            if cost < best_cost:
                best, best_cost = c, cost
        return best

    def _reacquire(self, cands: list[Candidate], tg: Target) -> Candidate | None:
        best, best_sim = None, -math.inf
        for c in cands:
            if not c.camera or math.hypot(c.x - tg.x, c.y - tg.y) > self.cfg.reacquire_radius:
                continue
            sim = similarity(tg.hist, c.hist) if tg.hist is not None else 1.0
            if sim >= self.cfg.reacquire_similarity and sim > best_sim:
                best, best_sim = c, sim
        return best
