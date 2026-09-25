"""Priority multiplexer for velocity commands. Pure Python, no ROS.

The highest-priority input that is still fresh wins. An input goes stale after its timeout,
so a crashed publisher can never keep the robot moving.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MuxInput:
    name: str
    priority: int
    timeout: float  # s


@dataclass
class _Slot:
    spec: MuxInput
    v: float = 0.0
    w: float = 0.0
    stamp: float | None = None


class CmdMux:
    def __init__(self, inputs: list[MuxInput]) -> None:
        names = [i.name for i in inputs]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate mux inputs: {names}")
        self._slots = {i.name: _Slot(i) for i in inputs}

    def update(self, name: str, v: float, w: float, now: float) -> None:
        slot = self._slots[name]
        slot.v, slot.w, slot.stamp = v, w, now

    def select(self, now: float) -> tuple[str | None, float, float]:
        """(active input or None, v, w). None → no fresh input, command is zero."""
        best: _Slot | None = None
        for slot in self._slots.values():
            if slot.stamp is None or now - slot.stamp > slot.spec.timeout:
                continue
            if best is None or slot.spec.priority > best.spec.priority:
                best = slot
        if best is None:
            return None, 0.0, 0.0
        return best.spec.name, best.v, best.w
