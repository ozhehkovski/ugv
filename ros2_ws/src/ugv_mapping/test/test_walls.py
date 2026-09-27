import math

import numpy as np
import pytest

from ugv_mapping.walls import flatten, load, parse_flat, rasterize, sample, save, to_frame, wall_across


def test_parse_and_flatten_roundtrip() -> None:
    segs = parse_flat([0, 0, 1, 0, 2, 2, 2, 3])
    assert segs == [(0.0, 0.0, 1.0, 0.0), (2.0, 2.0, 2.0, 3.0)]
    assert flatten(segs) == [0.0, 0.0, 1.0, 0.0, 2.0, 2.0, 2.0, 3.0]


@pytest.mark.parametrize("bad", [[0, 0, 1], [0, 0, float("nan"), 1], [0, 0, 100, 0]])
def test_parse_rejects_bad_input(bad: list) -> None:
    with pytest.raises(ValueError):
        parse_flat(bad)


def test_save_load(tmp_path) -> None:
    p = str(tmp_path / "walls.json")
    assert load(p) == []
    save(p, [(0.0, 0.0, 1.0, 0.5)])
    assert load(p) == [(0.0, 0.0, 1.0, 0.5)]


def test_sample_spacing_and_ends() -> None:
    pts = sample([(0.0, 0.0, 1.0, 0.0)], step=0.25)
    assert len(pts) == 5
    assert tuple(pts[0]) == (0.0, 0.0) and tuple(pts[-1]) == (1.0, 0.0)
    assert sample([]).shape == (0, 2)


def test_rasterize_marks_a_continuous_line() -> None:
    g = np.zeros((20, 20), dtype=np.int16)
    out = rasterize(g, [(0.05, 0.05, 0.95, 0.05)], origin=(0.0, 0.0), resolution=0.05)
    assert (out[1, 1:20] == 100).all()          # row = y 0.05, cols x 0.05 .. 0.95
    assert out.sum() == 100 * 19 and g.sum() == 0


def test_to_frame() -> None:
    p = to_frame(np.array([[1.0, 1.0]]), 1.0, 0.0, math.pi / 2)
    assert p[0] == pytest.approx([1.0, 0.0])      # the point is straight ahead of a robot facing +y


def test_wall_across_contact_ahead() -> None:
    x1, y1, x2, y2 = wall_across(0.0, 0.0, 0.0, contact_x=0.6, contact_y=0.0, half_len=0.3)
    assert (x1, y1, x2, y2) == pytest.approx((0.6, -0.3, 0.6, 0.3))
    x1, y1, x2, y2 = wall_across(1.0, 2.0, math.pi / 2, contact_x=0.6, contact_y=0.0, half_len=0.3)
    assert (x1, y1, x2, y2) == pytest.approx((1.3, 2.6, 0.7, 2.6))
