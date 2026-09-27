"""Full robot: base (drivers + EKF + safety) + persistent SLAM map + accessible terrain + Nav2 + web panel.

Web panel: http://<robot-ip>:8090
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("ugv_bringup")
    use_slam = LaunchConfiguration("use_slam")
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav.launch.py")),
        condition=IfCondition(LaunchConfiguration("use_nav")))

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "base.launch.py")),
        launch_arguments={"use_camera": LaunchConfiguration("use_camera")}.items())

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_slam", default_value="true"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_follow", default_value="true"),
        DeclareLaunchArgument("webui_port", default_value="8090"),
        base,
        nav,
        # map_manager starts/restarts slam_toolbox itself and keeps the map across restarts
        Node(package="ugv_mapping", executable="map_manager", name="map_manager", output="screen",
             condition=IfCondition(use_slam),
             parameters=[{"slam_params": os.path.join(pkg, "config", "slam_toolbox.yaml")}]),
        Node(package="ugv_mapping", executable="carry_detector", name="carry_detector", respawn=True, respawn_delay=2.0,
             output="screen", condition=IfCondition(use_slam)),
        Node(package="ugv_mapping", executable="virtual_walls", name="virtual_walls", respawn=True, respawn_delay=2.0,
             output="screen", condition=IfCondition(use_slam)),
        Node(package="ugv_mapping", executable="accessibility", name="accessibility", respawn=True, respawn_delay=2.0, output="screen",
             condition=IfCondition(use_slam)),
        # follow-me (idle until enabled from the web panel); needs the camera and the TensorRT engine
        Node(package="ugv_follow", executable="follow", name="follow", respawn=True, respawn_delay=3.0,
             output="screen", condition=IfCondition(LaunchConfiguration("use_follow"))),
        Node(package="ugv_webui", executable="webui", name="webui", respawn=True, respawn_delay=2.0, output="screen",
             parameters=[{"port": LaunchConfiguration("webui_port")}]),
    ])
