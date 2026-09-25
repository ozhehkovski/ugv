import pytest

from ugv_drivers.cmd_mux import CmdMux, MuxInput


def _mux() -> CmdMux:
    return CmdMux([
        MuxInput("teleop", priority=100, timeout=0.5),
        MuxInput("follow", priority=50, timeout=0.3),
        MuxInput("nav", priority=10, timeout=0.5),
    ])


def test_no_input_is_zero() -> None:
    assert _mux().select(0.0) == (None, 0.0, 0.0)


def test_higher_priority_wins() -> None:
    m = _mux()
    m.update("nav", 0.2, 0.0, 0.0)
    m.update("teleop", -0.1, 0.5, 0.0)
    assert m.select(0.1) == ("teleop", -0.1, 0.5)


def test_stale_input_falls_back_then_stops() -> None:
    m = _mux()
    m.update("nav", 0.2, 0.1, 0.0)
    m.update("teleop", 0.3, 0.0, 0.0)
    m.update("nav", 0.2, 0.1, 0.4)
    assert m.select(0.6)[0] == "nav"          # teleop timed out at 0.5
    assert m.select(1.0) == (None, 0.0, 0.0)  # nav timed out at 0.9


def test_duplicate_inputs_rejected() -> None:
    with pytest.raises(ValueError):
        CmdMux([MuxInput("a", 1, 1.0), MuxInput("a", 2, 1.0)])
