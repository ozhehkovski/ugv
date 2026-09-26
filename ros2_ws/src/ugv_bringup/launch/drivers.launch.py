"""Hardware layer: URDF/TF, VESC drive, RPLIDAR, BNO085, camera, footprint.

Publishes scan, odom, imu/data, camera/*, footprint and static TF; the drive listens on cmd_vel_safe.
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
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ugv_description"), "launch", "description.launch.py")))
    use_camera = LaunchConfiguration("use_camera")
    use_imu = LaunchConfiguration("use_imu")

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        description,
        Node(package="ugv_drivers", executable="vesc_driver", name="vesc_driver", respawn=True, respawn_delay=2.0, output="screen",
             remappings=[("cmd_vel", "cmd_vel_safe")]),
        Node(package="ugv_drivers", executable="rplidar", name="rplidar", respawn=True, respawn_delay=2.0, output="screen",
             parameters=[{"frame_id": "laser", "invert": True}]),
        Node(package="ugv_drivers", executable="bno085", name="bno085", respawn=True, respawn_delay=2.0, output="screen",
             condition=IfCondition(use_imu),
             parameters=[{"i2c_bus": 7, "address": 0x4A, "frame_id": "imu_link", "rate_hz": 50.0}]),
        Node(package="ugv_drivers", executable="camera", name="camera", respawn=True, respawn_delay=2.0, output="screen",
             condition=IfCondition(use_camera)),
        Node(package="ugv_drivers", executable="footprint_publisher", name="footprint_publisher", respawn=True, respawn_delay=2.0, output="screen"),
    ])
