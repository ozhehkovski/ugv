from ugv_mapping.carry import CarryDetector


def feed(det: CarryDetector, t0: float, secs: float, tilt: float, acc: float, driving: bool = False) -> tuple[bool, float]:
    t = t0
    while t < t0 + secs:
        if driving:
            det.command(t)
        det.update(t, tilt, acc)
        t += 0.02
    return det.carried, t


def test_lifting_is_a_carry_and_putting_down_ends_it() -> None:
    d = CarryDetector()
    assert feed(d, 0.0, 2.0, 0.0, 0.0)[0] is False
    carried, t = feed(d, 2.0, 0.5, 0.25, 0.0)           # tilted 14° while idle
    assert carried
    carried, t = feed(d, t, 1.0, 0.0, 0.0)              # held level briefly: still carried (hysteresis)
    assert carried
    carried, t = feed(d, t, 2.5, 0.0, 0.0)
    assert not carried


def test_walking_with_the_robot_level_is_a_carry() -> None:
    d = CarryDetector()
    carried, _ = feed(d, 0.0, 1.0, 0.0, 4.0)            # steps: vertical acceleration
    assert carried


def test_driving_over_a_threshold_is_not_a_carry() -> None:
    d = CarryDetector()
    carried, _ = feed(d, 0.0, 1.0, 0.2, 3.0, driving=True)
    assert not carried


def test_short_bump_is_not_a_carry() -> None:
    d = CarryDetector()
    carried, t = feed(d, 0.0, 0.2, 0.3, 0.0)
    carried, _ = feed(d, t, 1.0, 0.0, 0.0)
    assert not carried
