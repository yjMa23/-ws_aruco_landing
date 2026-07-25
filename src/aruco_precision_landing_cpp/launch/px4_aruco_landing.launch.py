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
                "vertical_velocity_feedforward_enabled",
                default_value="true",
                description="Enable P5C deck vertical-velocity feedforward during relative descent.",
            ),
            DeclareLaunchArgument(
                "vertical_velocity_feedforward_gain",
                default_value="1.0",
                description="Gain applied to estimated deck NED vertical velocity.",
            ),
            DeclareLaunchArgument(
                "vertical_velocity_feedforward_max_mps",
                default_value="0.60",
                description="Absolute vertical velocity feedforward limit.",
            ),
            DeclareLaunchArgument(
                "touchdown_detector_enabled",
                default_value="true",
                description="Enable P6A parallel touchdown evidence evaluation.",
            ),
            DeclareLaunchArgument(
                "touchdown_px4_status_timeout_s",
                default_value="0.20",
                description="Maximum accepted PX4 land-detector age.",
            ),
            DeclareLaunchArgument(
                "touchdown_visual_timeout_s",
                default_value="0.20",
                description="Maximum accepted visual-height age.",
            ),
            DeclareLaunchArgument(
                "touchdown_low_height_enter_m",
                default_value="0.18",
                description="Relative-height threshold for entering low-height evidence.",
            ),
            DeclareLaunchArgument(
                "touchdown_low_height_exit_m",
                default_value="0.28",
                description="Relative-height threshold for leaving low-height evidence.",
            ),
            DeclareLaunchArgument(
                "touchdown_max_relative_vertical_speed_mps",
                default_value="0.12",
                description="Maximum relative vertical speed accepted as low motion.",
            ),
            DeclareLaunchArgument(
                "touchdown_max_uav_vertical_speed_mps",
                default_value="0.15",
                description="Maximum UAV vertical speed accepted as low motion.",
            ),
            DeclareLaunchArgument(
                "touchdown_candidate_required_duration_s",
                default_value="0.50",
                description="Continuous candidate duration required for confirmation.",
            ),
            DeclareLaunchArgument(
                "final_descent_enabled",
                default_value="false",
                description="Explicitly enable P6B final descent below the P5B test height.",
            ),
            DeclareLaunchArgument(
                "final_descent_entry_height_m",
                default_value="0.50",
                description="Relative height at which P6B final descent is allowed to start.",
            ),
            DeclareLaunchArgument(
                "final_descent_rate_mps",
                default_value="0.03",
                description="P6B final relative-height descent rate.",
            ),
            DeclareLaunchArgument(
                "final_descent_minimum_command_height_m",
                default_value="0.15",
                description="Minimum relative-height command clamp before touchdown confirmation.",
            ),
            DeclareLaunchArgument(
                "final_descent_max_reference_tracking_error_m",
                default_value="0.20",
                description="Maximum relative-height tracking error before final descent pauses.",
            ),
            DeclareLaunchArgument(
                "relative_descent_enabled",
                default_value="false",
                description="Explicitly enable P5B relative-height descent to the test height.",
            ),
            DeclareLaunchArgument(
                "descent_minimum_test_height_m",
                default_value="0.50",
                description="Minimum relative-height reference allowed in the current SITL test.",
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
                "vehicle_land_detected_topic",
                default_value="/fmu/out/vehicle_land_detected",
                description="PX4 VehicleLandDetected output topic.",
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
                        "vertical_velocity_feedforward.enabled": ParameterValue(
                            LaunchConfiguration("vertical_velocity_feedforward_enabled"),
                            value_type=bool,
                        ),
                        "vertical_velocity_feedforward.deck_velocity_gain": ParameterValue(
                            LaunchConfiguration("vertical_velocity_feedforward_gain"),
                            value_type=float,
                        ),
                        "vertical_velocity_feedforward.max_abs_mps": ParameterValue(
                            LaunchConfiguration("vertical_velocity_feedforward_max_mps"),
                            value_type=float,
                        ),
                        "touchdown_detector.enabled": ParameterValue(
                            LaunchConfiguration("touchdown_detector_enabled"),
                            value_type=bool,
                        ),
                        "touchdown_detector.px4_status_timeout_s": ParameterValue(
                            LaunchConfiguration("touchdown_px4_status_timeout_s"),
                            value_type=float,
                        ),
                        "touchdown_detector.visual_timeout_s": ParameterValue(
                            LaunchConfiguration("touchdown_visual_timeout_s"),
                            value_type=float,
                        ),
                        "touchdown_detector.low_height_enter_m": ParameterValue(
                            LaunchConfiguration("touchdown_low_height_enter_m"),
                            value_type=float,
                        ),
                        "touchdown_detector.low_height_exit_m": ParameterValue(
                            LaunchConfiguration("touchdown_low_height_exit_m"),
                            value_type=float,
                        ),
                        "touchdown_detector.max_relative_vertical_speed_mps": ParameterValue(
                            LaunchConfiguration(
                                "touchdown_max_relative_vertical_speed_mps"
                            ),
                            value_type=float,
                        ),
                        "touchdown_detector.max_uav_vertical_speed_mps": ParameterValue(
                            LaunchConfiguration("touchdown_max_uav_vertical_speed_mps"),
                            value_type=float,
                        ),
                        "touchdown_detector.candidate_required_duration_s": ParameterValue(
                            LaunchConfiguration(
                                "touchdown_candidate_required_duration_s"
                            ),
                            value_type=float,
                        ),
                        "final_descent.enabled": ParameterValue(
                            LaunchConfiguration("final_descent_enabled"),
                            value_type=bool,
                        ),
                        "final_descent.entry_height_m": ParameterValue(
                            LaunchConfiguration("final_descent_entry_height_m"),
                            value_type=float,
                        ),
                        "final_descent.rate_mps": ParameterValue(
                            LaunchConfiguration("final_descent_rate_mps"),
                            value_type=float,
                        ),
                        "final_descent.minimum_command_height_m": ParameterValue(
                            LaunchConfiguration(
                                "final_descent_minimum_command_height_m"
                            ),
                            value_type=float,
                        ),
                        "final_descent.max_reference_tracking_error_m": ParameterValue(
                            LaunchConfiguration(
                                "final_descent_max_reference_tracking_error_m"
                            ),
                            value_type=float,
                        ),
                        "descent.enabled": ParameterValue(
                            LaunchConfiguration("relative_descent_enabled"),
                            value_type=bool,
                        ),
                        "descent.minimum_test_height_m": ParameterValue(
                            LaunchConfiguration("descent_minimum_test_height_m"),
                            value_type=float,
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
                        "/fmu/out/vehicle_land_detected",
                        LaunchConfiguration("vehicle_land_detected_topic"),
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
