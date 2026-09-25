import math

import pytest

from ugv_drivers.heading_hold import HeadingHold


def hold() -> HeadingHold:
    return HeadingHold(k_rate=0.5, k_heading=2.0, max_correction=0.3, max_heading_error=0.35)


def test_inactive_resets_and_gives_no_correction() -> None:
    h = hold()
    h.update(0.0, 0.2, 0.1, active=True)
    assert h.update(0.0, 0.2, 0.1, active=False) == 0.0
    assert h.heading_error == 0.0


def test_drift_to_the_right_is_corrected_to_the_left() -> None:
    h = hold()
    corr = 0.0
    for _ in range(10):                      # robot yaws right at 0.1 rad/s for 1 s
        corr = h.update(0.0, -0.1, 0.1, active=True)
    assert h.heading_error == pytest.approx(0.1)
    assert corr > 0.0                        # steer left (CCW)


def test_correction_is_clamped() -> None:
    h = hold()
    for _ in range(100):
        corr = h.update(0.0, -1.0, 0.1, active=True)
    assert corr == pytest.approx(0.3)
    assert h.heading_error == pytest.approx(0.35)


def test_closed_loop_straight_run_returns_to_heading() -> None:
    """Simple plant: casters push the robot with a yaw disturbance; the robot yaws at w_cmd + disturbance."""
    h = hold()
    dt = 0.033
    heading, w_prev = 0.0, 0.0
    for k in range(int(6 / dt)):
        disturbance = -0.15 if k * dt < 1.0 else 0.0     # caster flip at the start
        w_actual = w_prev + disturbance
        heading += w_actual * dt
        w_prev = h.update(0.0, w_actual, dt, active=True)
    assert abs(heading) < math.radians(1.0)
