import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock.",
            ),
            DeclareLaunchArgument(
                "tracking_mode",
                default_value="PREDICTED_POSITION_VELOCITY_FF",
                description="Moving-target tracking control mode.",
            ),
            DeclareLaunchArgument(
                "prediction_horizon_s",
                default_value="0.10",
                description="Additional constant-velocity prediction horizon in seconds.",
            ),
            DeclareLaunchArgument(
                "velocity_feedforward_gain",
                default_value="1.0",
                description="Deck velocity feedforward gain.",
            ),
            DeclareLaunchArgument(
                "relative_velocity_gain",
                default_value="0.25",
                description="Relative velocity damping gain.",
            ),
            DeclareLaunchArgument(
                "adaptive_relative_velocity_gain_enabled",
                default_value="true",
                description="Enable acceleration-aware relative velocity gain scheduling.",
            ),
            DeclareLaunchArgument(
                "adaptive_gain_min",
                default_value="0.25",
                description="Minimum scheduled relative velocity gain.",
            ),
            DeclareLaunchArgument(
                "adaptive_gain_max",
                default_value="1.2",
                description="Maximum scheduled relative velocity gain.",
            ),
            DeclareLaunchArgument(
                "adaptive_acceleration_low_threshold_mps2",
                default_value="0.05",
                description="Acceleration threshold below which minimum gain is used.",
            ),
            DeclareLaunchArgument(
                "adaptive_acceleration_high_threshold_mps2",
                default_value="0.35",
                description="Acceleration threshold above which maximum gain is used.",
            ),
            DeclareLaunchArgument(
                "adaptive_max_acceleration_mps2",
                default_value="1.50",
                description="Maximum acceleration magnitude accepted by the scheduler.",
            ),
            DeclareLaunchArgument(
                "adaptive_acceleration_filter_gain",
                default_value="0.20",
                description="First-order acceleration filter gain.",
            ),
            DeclareLaunchArgument(
                "relative_descent_enabled",
                default_value="false",
                description="Explicitly enable P5B relative-height descent to the test height.",
            ),
            DeclareLaunchArgument(
                "landing_window_minimum_relative_height_m",
                default_value="0.20",
                description="Minimum valid estimated relative height for landing-window evaluation.",
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
            DeclareLaunchArgument(
                "deck_gps_fix_topic",
                default_value="/deck/gps/fix",
                description="Deck GNSS NavSatFix topic.",
            ),
            DeclareLaunchArgument(
                "deck_gps_velocity_topic",
                default_value="/deck/gps/velocity",
                description="Deck GNSS ENU velocity topic.",
            ),
            Node(
                package="aruco_precision_landing_cpp",
                executable="px4_aruco_landing_node",
                name="px4_aruco_landing_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "tracking.mode": ParameterValue(
                            LaunchConfiguration("tracking_mode"), value_type=str
                        ),
                        "motion_predictor.additional_prediction_horizon_s": ParameterValue(
                            LaunchConfiguration("prediction_horizon_s"), value_type=float
                        ),
                        "tracking.velocity_feedforward_gain": ParameterValue(
                            LaunchConfiguration("velocity_feedforward_gain"),
                            value_type=float,
                        ),
                        "tracking.relative_velocity_gain": ParameterValue(
                            LaunchConfiguration("relative_velocity_gain"), value_type=float
                        ),
                        "tracking.adaptive_relative_velocity_gain.enabled": ParameterValue(
                            LaunchConfiguration("adaptive_relative_velocity_gain_enabled"),
                            value_type=bool,
                        ),
                        "tracking.adaptive_relative_velocity_gain.min_gain": ParameterValue(
                            LaunchConfiguration("adaptive_gain_min"), value_type=float
                        ),
                        "tracking.adaptive_relative_velocity_gain.max_gain": ParameterValue(
                            LaunchConfiguration("adaptive_gain_max"), value_type=float
                        ),
                        "tracking.adaptive_relative_velocity_gain.acceleration_low_threshold_mps2": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_acceleration_low_threshold_mps2"
                            ),
                            value_type=float,
                        ),
                        "tracking.adaptive_relative_velocity_gain.acceleration_high_threshold_mps2": ParameterValue(
                            LaunchConfiguration(
                                "adaptive_acceleration_high_threshold_mps2"
                            ),
                            value_type=float,
                        ),
                        "tracking.adaptive_relative_velocity_gain.max_acceleration_mps2": ParameterValue(
                            LaunchConfiguration("adaptive_max_acceleration_mps2"),
                            value_type=float,
                        ),
                        "tracking.adaptive_relative_velocity_gain.acceleration_filter_gain": ParameterValue(
                            LaunchConfiguration("adaptive_acceleration_filter_gain"),
                            value_type=float,
                        ),
                        "descent.enabled": ParameterValue(
                            LaunchConfiguration("relative_descent_enabled"),
                            value_type=bool,
                        ),
                        "landing_window.minimum_relative_height_m": ParameterValue(
                            LaunchConfiguration(
                                "landing_window_minimum_relative_height_m"
                            ),
                            value_type=float,
                        ),
                    },
                ],
                remappings=[
                    (
                        "/fmu/out/vehicle_status",
                        LaunchConfiguration("vehicle_status_topic"),
                    ),
                    (
                        "/fmu/out/vehicle_local_position",
                        LaunchConfiguration("vehicle_local_position_topic"),
                    ),
                    (
                        "/deck/gps/fix",
                        LaunchConfiguration("deck_gps_fix_topic"),
                    ),
                    (
                        "/deck/gps/velocity",
                        LaunchConfiguration("deck_gps_velocity_topic"),
                    ),
                ],
            ),
        ]
    )
