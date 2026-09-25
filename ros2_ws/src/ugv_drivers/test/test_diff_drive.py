import math

import pytest

from ugv_drivers.diff_drive import (
    apply_min_wheel_speed,
    DriveLimits,
    Odometry2D,
    VelocityLimiter,
    body_to_wheels,
    erpm_per_mps,
    meters_per_tach,
    ramp,
)

LIMITS = DriveLimits(max_linear=0.55, max_angular=0.9, max_wheel_speed=0.55,
                     linear_accel=0.3, linear_decel=1.0, angular_accel=1.5)


def test_ramp_uses_accel_when_speeding_up_and_decel_when_slowing() -> None:
    assert ramp(0.0, 1.0, accel=0.3, decel=1.0, dt=0.1) == pytest.approx(0.03)
    assert ramp(0.5, 0.0, accel=0.3, decel=1.0, dt=0.1) == pytest.approx(0.4)
    assert ramp(-0.5, 0.0, accel=0.3, decel=1.0, dt=0.1) == pytest.approx(-0.4)
    # reversing: first brake with decel
    assert ramp(0.05, -1.0, accel=0.3, decel=1.0, dt=0.1) == pytest.approx(-0.05)


def test_ramp_reaches_target_without_overshoot() -> None:
    assert ramp(0.29, 0.3, accel=0.3, decel=1.0, dt=0.1) == 0.3


def test_limiter_never_exceeds_2kmh_and_starts_softly() -> None:
    lim = VelocityLimiter(LIMITS)
    v, _ = lim.step(5.0, 0.0, 0.1)
    assert v == pytest.approx(0.03)                    # soft start: 0.3 m/s² · 0.1 s
    for _ in range(200):
        v, w = lim.step(5.0, 5.0, 0.1)
    assert v == pytest.approx(0.55) and w == pytest.approx(0.9)


def test_limiter_time_to_full_speed() -> None:
    lim = VelocityLimiter(LIMITS)
    steps = 0
    while lim.step(0.55, 0.0, 0.05)[0] < 0.55:
        steps += 1
    assert steps * 0.05 == pytest.approx(0.55 / 0.3, abs=0.06)


def test_body_to_wheels_tank_turn() -> None:
    left, right = body_to_wheels(0.0, 1.0, track=0.5, max_wheel_speed=0.55)
    assert left == pytest.approx(-0.25) and right == pytest.approx(0.25)


def test_body_to_wheels_saturation_preserves_curvature() -> None:
    left, right = body_to_wheels(0.55, 0.9, track=0.515, max_wheel_speed=0.55)
    assert max(abs(left), abs(right)) == pytest.approx(0.55)
    # curvature w/v is unchanged
    v, w = (left + right) / 2, (right - left) / 0.515
    assert w / v == pytest.approx(0.9 / 0.55)


def test_conversions_match_bench_measurement() -> None:
    # bench: 300 ERPM ≈ 0.13 m/s, 120 tach counts per wheel revolution
    k = erpm_per_mps(0.0825, 20)
    assert 300 / k == pytest.approx(0.1296, abs=1e-3)
    assert meters_per_tach(0.0825, 20) * 120 == pytest.approx(2 * math.pi * 0.0825)


def test_odometry_straight_and_spin() -> None:
    odo = Odometry2D(track=0.5)
    odo.update(1.0, 1.0)
    assert (odo.x, odo.y, odo.theta) == pytest.approx((1.0, 0.0, 0.0))
    quarter = math.pi / 2 * 0.25
    odo.update(-quarter, quarter)
    assert odo.theta == pytest.approx(math.pi / 2)
    assert odo.x == pytest.approx(1.0)


def test_min_wheel_speed_lifts_slow_commands_keeping_curvature() -> None:
    left, right = apply_min_wheel_speed(-0.02, 0.02, 0.06)      # slow tank turn
    assert (left, right) == pytest.approx((-0.06, 0.06))
    left, right = apply_min_wheel_speed(0.01, 0.03, 0.06)
    assert (left, right) == pytest.approx((0.02, 0.06))


def test_min_wheel_speed_keeps_zero_and_fast_commands() -> None:
    assert apply_min_wheel_speed(0.0, 0.0, 0.06) == (0.0, 0.0)
    assert apply_min_wheel_speed(0.1, 0.2, 0.06) == (0.1, 0.2)
