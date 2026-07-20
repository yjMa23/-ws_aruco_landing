import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("moving_deck_sim")
    default_config = os.path.join(package_share, "config", "gnss_ideal.yaml")
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to a deck GNSS sensor YAML file.",
            ),
            Node(
                package="moving_deck_sim",
                executable="deck_gnss_simulator",
                name="deck_gnss_simulator",
                parameters=[config_file, {"use_sim_time": True}],
                output="screen",
            ),
        ]
    )
