"""Nav2 navigation (needs base + SLAM running): plan on the map, follow with MPPI.

Velocity output → cmd_vel/nav (lowest-priority mux input), so teleop and the safety chain win.
Goals: the web panel (click on the map) or the NavigateToPose action.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(get_package_share_directory("ugv_bringup"), "config", "nav2.yaml")
    to_nav = [("cmd_vel", "cmd_vel/nav")]

    def nav_node(package: str, executable: str, name: str, remap: list | None = None) -> Node:
        return Node(package=package, executable=executable, name=name, output="screen",
                    parameters=[params], remappings=remap or [])

    return LaunchDescription([
        nav_node("nav2_controller", "controller_server", "controller_server", to_nav),
        nav_node("nav2_smoother", "smoother_server", "smoother_server"),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node("nav2_behaviors", "behavior_server", "behavior_server", to_nav),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        nav_node("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        nav_node("nav2_lifecycle_manager", "lifecycle_manager", "lifecycle_manager_navigation"),
    ])
