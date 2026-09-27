import pytest

from ugv_drivers.bump import BumpConfig, BumpDetector, contact_point


def run(det: BumpDetector, secs: float, targets, measured, currents, t0: float = 0.0, accel=None):
    t, ev = t0, None
    while t < t0 + secs and ev is None:
        ev = det.update(t, targets, measured, currents, accel)
        t += 0.033
    return ev, t


def test_pushing_against_a_mirror_is_detected() -> None:
    det = BumpDetector(BumpConfig())
    ev, t = run(det, 1.0, (0.2, 0.2), (0.01, 0.02), (6.0, 6.5))
    assert ev in ("stall_left", "stall_right")
    assert 0.4 <= t <= 0.5


def test_normal_driving_and_soft_start_are_not_bumps() -> None:
    det = BumpDetector(BumpConfig())
    assert run(det, 3.0, (0.3, 0.3), (0.29, 0.31), (1.5, 1.4))[0] is None          # cruising
    assert run(det, 3.0, (0.1, 0.1), (0.02, 0.02), (1.0, 1.0))[0] is None          # slow start, low current
    assert run(det, 0.3, (0.2, 0.2), (0.0, 0.0), (6.0, 6.0))[0] is None            # breakaway shorter than 0.4 s


def test_stall_must_be_continuous() -> None:
    det = BumpDetector(BumpConfig())
    for k in range(60):                  # stall and free alternate every 0.2 s
        stalled = (k // 6) % 2 == 0
        meas = (0.0, 0.0) if stalled else (0.2, 0.2)
        assert det.update(k * 0.033, (0.2, 0.2), meas, (6.0, 6.0)) is None


def test_cooldown_between_events() -> None:
    det = BumpDetector(BumpConfig(cooldown=2.0))
    ev, t = run(det, 1.0, (0.2, 0.2), (0.0, 0.0), (6.0, 6.0))
    assert ev is not None
    assert run(det, 1.0, (0.2, 0.2), (0.0, 0.0), (6.0, 6.0), t0=t)[0] is None
    assert run(det, 1.0, (0.2, 0.2), (0.0, 0.0), (6.0, 6.0), t0=t + 2.1)[0] is not None


def test_impact_rule_only_when_enabled_and_against_motion() -> None:
    off = BumpDetector(BumpConfig())
    assert off.update(0.0, (0.3, 0.3), (0.3, 0.3), (1.0, 1.0), accel_x=-20.0) is None
    on = BumpDetector(BumpConfig(impact_accel=8.0))
    assert on.update(0.0, (0.3, 0.3), (0.3, 0.3), (1.0, 1.0), accel_x=+12.0) is None   # pushed forward
    assert on.update(0.1, (0.3, 0.3), (0.3, 0.3), (1.0, 1.0), accel_x=-12.0) == "impact"


def test_contact_point() -> None:
    kw = dict(track=0.484, front=0.555, rear=-0.065, half_width=0.28)
    assert contact_point(0.2, 0.2, **kw) == (0.555, 0.0)
    assert contact_point(-0.1, -0.1, **kw) == (-0.065, 0.0)
    assert contact_point(-0.1, 0.1, **kw) == pytest.approx((0.555, 0.28))     # CCW spin: left nose corner
    assert contact_point(0.1, -0.1, **kw) == pytest.approx((0.555, -0.28))
