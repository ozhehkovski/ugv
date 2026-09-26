"""Nav2 config must agree with robot.yaml (footprint) and the safety caps."""
import ast
import math
import os

import pytest
import yaml

from ugv_drivers.robot_config import footprint_polygon, load_robot_config, turn_sweep_radius

SRC = os.path.join(os.path.dirname(__file__), "..", "..")
ROBOT = load_robot_config(os.path.join(SRC, "ugv_description", "config", "robot.yaml"))
NAV2 = yaml.safe_load(open(os.path.join(SRC, "ugv_bringup", "config", "nav2.yaml"), encoding="utf-8"))


def costmap(name: str) -> dict:
    return NAV2[name][name]["ros__parameters"]


@pytest.mark.parametrize("name", ["local_costmap", "global_costmap"])
def test_footprint_matches_robot_yaml(name: str) -> None:
    fp = ast.literal_eval(costmap(name)["footprint"])
    expected = footprint_polygon(ROBOT)
    assert len(fp) == len(expected)
    for (x, y), (ex, ey) in zip(fp, expected):
        assert x == pytest.approx(ex, abs=1e-3) and y == pytest.approx(ey, abs=1e-3)


@pytest.mark.parametrize("name", ["local_costmap", "global_costmap"])
def test_inflation_covers_circumscribed_radius(name: str) -> None:
    # Smac with a non-circular footprint needs inflation ≥ circumscribed radius
    assert costmap(name)["inflation_layer"]["inflation_radius"] >= turn_sweep_radius(ROBOT)


def test_controller_speeds_within_hard_limits() -> None:
    mppi = NAV2["controller_server"]["ros__parameters"]["FollowPath"]
    assert mppi["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert mppi["rotate_to_heading_angular_vel"] <= ROBOT["limits"]["max_angular"]
    assert mppi["vx_max"] <= ROBOT["limits"]["max_linear"]
    assert mppi["wz_max"] <= ROBOT["limits"]["max_angular"]
    assert mppi["CostCritic"]["consider_footprint"] is True
    assert mppi["motion_model"] == "DiffDrive"
    assert math.isclose(mppi["time_steps"] * mppi["model_dt"], 2.5)
