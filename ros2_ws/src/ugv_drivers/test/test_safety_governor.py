import numpy as np
import pytest

from ugv_drivers.safety_governor import Body, SafetyGovernor

BODY = Body(front=0.555, rear=-0.065, half_width=0.28)


def gov(**kw) -> SafetyGovernor:
    return SafetyGovernor(BODY, stop_margin=0.05, horizon=1.5, dt=0.05, track=0.515, **kw)


def wall_ahead(gap: float) -> np.ndarray:
    ys = np.linspace(-1.0, 1.0, 81)
    return np.column_stack((np.full_like(ys, BODY.front + gap), ys))


def wall_right(gap: float) -> np.ndarray:
    xs = np.linspace(-0.5, 1.2, 69)
    return np.column_stack((xs, np.full_like(xs, -(BODY.half_width + gap))))


def test_free_space_passes_unchanged() -> None:
    d = gov().limit(0.4, 0.2, np.empty((0, 2)))
    assert (d.v, d.w, d.reason) == (0.4, 0.2, "ok")


def test_far_wall_ahead_passes_near_wall_slows() -> None:
    assert gov().limit(0.3, 0.0, wall_ahead(2.0)).reason == "ok"
    d = gov().limit(0.3, 0.0, wall_ahead(0.30))
    assert d.reason == "slow" and 0.0 < d.v < 0.3


def test_wall_inside_margin_blocks_forward_but_allows_reverse() -> None:
    pts = wall_ahead(0.03)
    assert gov().limit(0.2, 0.0, pts).reason == "blocked"
    d = gov().limit(-0.15, 0.0, pts)
    assert d.reason == "ok" and d.v == pytest.approx(-0.15)


def test_never_trapped_next_to_a_wall() -> None:
    """The case from the floor test: obstacle 1–3 cm off the right side of the nose."""
    pts = wall_right(0.015)
    g = gov()
    assert g.limit(0.0, -0.4, pts).reason == "blocked"      # nose corner would swing into it
    assert g.limit(0.0, 0.4, pts).reason == "ok"            # turning away is fine
    assert g.limit(0.2, 0.0, pts).reason == "ok"            # sliding along it is fine
    assert g.limit(-0.15, 0.0, pts).reason == "ok"


def test_turn_toward_wall_is_limited_before_contact() -> None:
    d = gov().limit(0.0, -0.5, wall_right(0.15))
    assert d.reason in ("slow", "blocked") and abs(d.w) < 0.5


def test_points_inside_body_are_ignored() -> None:
    d = gov().limit(0.3, 0.0, np.array([[0.3, 0.0], [0.1, 0.1]]))
    assert d.reason == "ok"


def test_min_wheel_speed_lift_cannot_push_into_obstacle() -> None:
    # 7.5 cm from the wall a crawl of ~0.01 m/s is still safe, but the driver would lift it
    # to 0.06 m/s → the governor must block instead
    assert gov().limit(0.3, 0.0, wall_ahead(0.075)).reason == "slow"
    assert gov(min_wheel_speed=0.06).limit(0.3, 0.0, wall_ahead(0.075)).reason == "blocked"


def test_idle_command() -> None:
    assert gov().limit(0.0, 0.0, wall_ahead(0.01)).reason == "idle"


ESCAPE = [(v, w) for v in (0.12, 0.0, -0.1) for w in (0.4, 0.0, -0.4) if (v, w) != (0.0, 0.0)]


def test_escape_from_wall_behind_left_prefers_gaining_motion() -> None:
    """The hallway case: wall 1-3 cm behind the rear-left corner."""
    ys = np.linspace(0.0, 0.6, 25)
    pts = np.column_stack((np.full_like(ys, BODY.rear - 0.02), ys))
    best = gov().escape_command(pts, ESCAPE)
    assert best is not None
    v, w, clr = best
    assert v > 0                                  # pull away forward
    assert clr > 0.05


def test_escape_none_when_nothing_helps_and_none_needed_far_away() -> None:
    assert gov().escape_command(np.empty((0, 2)), ESCAPE) is None
