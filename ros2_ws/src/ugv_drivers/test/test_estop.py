from ugv_drivers.estop import EstopLatch


def test_idle_latch_allows_motion() -> None:
    assert EstopLatch().allow(0.2, 0.0, stale=False)


def test_engaged_blocks_everything() -> None:
    latch = EstopLatch()
    latch.set(True)
    assert not latch.allow(0.0, 0.0, stale=True)
    assert latch.state == "estop"


def test_release_with_held_joystick_does_not_move() -> None:
    latch = EstopLatch()
    latch.set(True)
    latch.set(False)
    assert not latch.allow(0.3, 0.0, stale=False)
    assert latch.state == "rearm"
    assert latch.allow(0.0, 0.0, stale=False)      # joystick released → re-armed
    assert latch.allow(0.3, 0.0, stale=False)
    assert latch.state == "ok"


def test_silence_rearms() -> None:
    latch = EstopLatch()
    latch.set(True)
    latch.set(False)
    assert latch.allow(0.3, 0.2, stale=True)
