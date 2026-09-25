import math

import numpy as np
import pytest

from ugv_webui.map_render import FREE, OCCUPIED, UNKNOWN, occupancy_to_image
from ugv_webui.teleop_gate import TeleopGate


def test_occupancy_colors_and_vertical_flip() -> None:
    # 2x2 grid, row-major from the map origin (bottom-left): [bottom-left, bottom-right, top-left, top-right]
    img = occupancy_to_image([-1, 0, 100, 50], width=2, height=2)
    assert img.dtype == np.uint8 and img.shape == (2, 2)
    assert img[1, 0] == UNKNOWN and img[1, 1] == FREE      # bottom row of the map → last image row
    assert img[0, 0] == OCCUPIED and OCCUPIED < img[0, 1] < FREE


def test_occupancy_size_mismatch() -> None:
    with pytest.raises(ValueError):
        occupancy_to_image([0, 0, 0], width=2, height=2)


def test_gate_open_while_updates_arrive_and_clamped() -> None:
    gate = TeleopGate(timeout=0.3, max_linear=0.5, max_angular=0.8)
    gate.set(2.0, -3.0, now=10.0)
    assert gate.current(10.2) == (0.5, -0.8)


def test_gate_closes_when_updates_stop() -> None:
    gate = TeleopGate(timeout=0.3, max_linear=0.5, max_angular=0.8)
    assert gate.current(0.0) is None
    gate.set(0.2, 0.0, now=1.0)
    assert gate.current(1.31) is None


def test_gate_cancel_and_nan() -> None:
    gate = TeleopGate(timeout=0.3, max_linear=0.5, max_angular=0.8)
    gate.set(0.2, 0.1, now=1.0)
    gate.cancel()
    assert gate.current(1.0) is None
    with pytest.raises(ValueError):
        gate.set(math.nan, 0.0, now=1.0)
