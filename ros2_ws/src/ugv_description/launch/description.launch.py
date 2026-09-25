"""robot_state_publisher with the UGV URDF (static TF of all sensors)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    xacro_file = os.path.join(get_package_share_directory("ugv_description"), "urdf", "ugv.urdf.xacro")
    prefix = LaunchConfiguration("prefix")
    robot_description = ParameterValue(Command(["xacro ", xacro_file, " prefix:=", prefix]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value="", description="TF frame prefix (multi-robot)"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ])
