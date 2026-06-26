import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("aruco_precision_landing_cpp")
    default_config = os.path.join(
        package_share, "config", "px4_aruco_landing.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the PX4 ArUco landing parameter file.",
            ),
            DeclareLaunchArgument(
                "vehicle_status_topic",
                default_value="/fmu/out/vehicle_status_v4",
                description="PX4 VehicleStatus output topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_local_position_topic",
                default_value="/fmu/out/vehicle_local_position_v1",
                description="PX4 VehicleLocalPosition output topic.",
            ),
            Node(
                package="aruco_precision_landing_cpp",
                executable="px4_aruco_landing_node",
                name="px4_aruco_landing_node",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
                remappings=[
                    (
                        "/fmu/out/vehicle_status",
                        LaunchConfiguration("vehicle_status_topic"),
                    ),
                    (
                        "/fmu/out/vehicle_local_position",
                        LaunchConfiguration("vehicle_local_position_topic"),
                    ),
                ],
            ),
        ]
    )
