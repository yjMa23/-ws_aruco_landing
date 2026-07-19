import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("aruco_detector")
    default_config = os.path.join(package_share, "config", "aruco_detector.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the aruco_detector parameter file.",
            ),
            Node(
                package="aruco_detector",
                executable="aruco_detector_node",
                name="aruco_detector",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
