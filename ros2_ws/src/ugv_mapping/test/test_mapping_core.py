import json
import os

import numpy as np
import pytest

from ugv_mapping.accessibility import BLOCKED, PASSABLE, TURNABLE, UNKNOWN, GridInfo, accessible_layer
from ugv_mapping.map_store import ActiveState, MapStore, auto_name, valid_name

RES = 0.05


def room(w_m: float, h_m: float, walls: list[tuple[float, float, float, float]] = ()) -> tuple[list[int], GridInfo]:
    """Free room with a 1-cell wall border, unknown outside; extra wall rectangles in metres."""
    w, h = int(w_m / RES) + 2, int(h_m / RES) + 2
    g = np.full((h, w), 100, dtype=np.int16)
    g[1:-1, 1:-1] = 0
    for x0, y0, x1, y1 in walls:
        g[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100
    return g.ravel().tolist(), GridInfo(w, h, RES, 0.0, 0.0)


def layer_at(layer: np.ndarray, info: GridInfo, x: float, y: float) -> int:
    return int(layer[info.cell(x, y)])


def test_big_room_center_is_turnable_edges_blocked() -> None:
    data, info = room(4.0, 4.0)
    L = accessible_layer(data, info, (2.0, 2.0), half_width=0.28, turn_radius=0.62, margin=0.05)
    assert layer_at(L, info, 2.0, 2.0) == TURNABLE
    assert layer_at(L, info, 0.5, 2.0) == PASSABLE        # 0.5 m from the wall: fits, cannot spin
    assert layer_at(L, info, 0.2, 2.0) == BLOCKED         # too close to the wall


def test_narrow_door_splits_rooms() -> None:
    # two 3x3 rooms joined by a wall at x=3 with a 0.5 m door (narrower than 0.56 + 2*margin)
    data, info = room(6.0, 3.0, walls=[(3.0, 0.0, 3.1, 1.25), (3.0, 1.75, 3.1, 3.0)])
    L = accessible_layer(data, info, (1.5, 1.5), half_width=0.28, turn_radius=0.62, margin=0.05)
    assert layer_at(L, info, 1.5, 1.5) == TURNABLE
    assert layer_at(L, info, 4.5, 1.5) == BLOCKED         # free but unreachable through the door


def test_wide_door_connects_rooms() -> None:
    data, info = room(6.0, 3.0, walls=[(3.0, 0.0, 3.1, 1.0), (3.0, 2.0, 3.1, 3.0)])   # 1.0 m door
    L = accessible_layer(data, info, (1.5, 1.5), half_width=0.28, turn_radius=0.62, margin=0.05)
    assert layer_at(L, info, 4.5, 1.5) == TURNABLE


def test_robot_parked_at_wall_still_seeds() -> None:
    data, info = room(4.0, 4.0)
    L = accessible_layer(data, info, (0.15, 2.0), half_width=0.28, turn_radius=0.62, margin=0.05)
    assert layer_at(L, info, 2.0, 2.0) == TURNABLE


def test_unknown_stays_unknown_and_no_pose() -> None:
    data, info = room(2.0, 2.0)
    L = accessible_layer(data, info, None, half_width=0.28, turn_radius=0.62, margin=0.05)
    assert (L != PASSABLE).all() and int(L[0, 0]) == BLOCKED
    data = [-1] * (10 * 10)
    L = accessible_layer(data, GridInfo(10, 10, RES, 0, 0), (0.2, 0.2), 0.28, 0.62, 0.05)
    assert (L == UNKNOWN).all()


def test_names() -> None:
    assert valid_name("kitchen_1") and valid_name("map-20260926-1700")
    assert not valid_name("../etc") and not valid_name("a b") and not valid_name("")
    assert auto_name().startswith("map-")


def test_store_active_roundtrip_and_listing(tmp_path) -> None:
    store = MapStore(str(tmp_path))
    assert store.read_active() == ActiveState()
    store.write_active(ActiveState("room", [1.0, -2.0, 0.5]))
    assert store.read_active() == ActiveState("room", [1.0, -2.0, 0.5])
    os.makedirs(store.dir("room"))
    open(store.base("room") + ".posegraph", "w").close()
    os.makedirs(store.dir("empty"))                      # no posegraph → not listed
    assert store.list() == ["room"]
    store.copy("room", "room2")
    assert store.list() == ["room", "room2"]
    with pytest.raises(ValueError):
        store.copy("room", "room2")
    with pytest.raises(ValueError):
        store.dir("../x")


def test_store_rejects_corrupt_active(tmp_path) -> None:
    store = MapStore(str(tmp_path))
    with open(os.path.join(str(tmp_path), "active.json"), "w") as f:
        f.write("{broken")
    with pytest.raises(ValueError):
        store.read_active()
    with open(os.path.join(str(tmp_path), "active.json"), "w") as f:
        json.dump({"map": "../../x", "pose": [0, 0, 0]}, f)
    with pytest.raises(ValueError):
        store.read_active()


def test_sparse_lidar_speckles_do_not_block() -> None:
    """Unknown single cells between lidar rays must not close the free space."""
    data, info = room(4.0, 4.0)
    g = np.array(data).reshape(info.height, info.width)
    rng = np.random.default_rng(0)
    holes = (rng.random(g.shape) < 0.25) & (g == 0)
    g[holes] = -1
    L = accessible_layer(g.ravel().tolist(), info, (2.0, 2.0), 0.28, 0.62, 0.05)
    assert layer_at(L, info, 2.0, 2.0) == TURNABLE
    assert layer_at(L, info, 0.5, 2.0) == PASSABLE


def test_large_unexplored_area_is_not_passable() -> None:
    data, info = room(4.0, 4.0)
    g = np.array(data).reshape(info.height, info.width)
    g[:, int(2.5 / RES):-1] = -1                  # right part never seen
    L = accessible_layer(g.ravel().tolist(), info, (1.0, 2.0), 0.28, 0.62, 0.05)
    assert layer_at(L, info, 1.0, 2.0) in (PASSABLE, TURNABLE)
    assert layer_at(L, info, 3.5, 2.0) == UNKNOWN
