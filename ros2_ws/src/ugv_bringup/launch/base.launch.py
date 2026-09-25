"""Base robot: drivers + EKF (odom→base_footprint) + velocity safety chain.

Drive it:  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel/teleop
Stop it:   latched std_msgs/Bool on /estop from a live publisher (see scripts/estop_test.py)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("ugv_bringup")
    safety = os.path.join(pkg, "config", "safety.yaml")
    ekf = os.path.join(pkg, "config", "ekf.yaml")

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "drivers.launch.py")),
        launch_arguments={"use_camera": LaunchConfiguration("use_camera")}.items())

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        drivers,
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node", output="screen",
             parameters=[ekf]),
        Node(package="ugv_drivers", executable="cmd_mux", name="cmd_mux", output="screen",
             parameters=[safety], remappings=[("cmd_vel_out", "cmd_vel_mux")]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[safety],
             remappings=[("cmd_vel", "cmd_vel_mux"), ("cmd_vel_smoothed", "cmd_vel_smoothed")]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             output="screen", parameters=[safety]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_safety",
             output="screen", parameters=[safety]),
    ])
