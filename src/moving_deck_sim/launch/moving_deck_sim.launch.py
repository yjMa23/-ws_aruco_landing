import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, package_share: str, gz_launch: str, gui_config_path: str):
    environment = LaunchConfiguration("environment").perform(context)
    if environment not in ("legacy", "marine"):
        raise RuntimeError("environment must be 'legacy' or 'marine'")

    config_file = LaunchConfiguration("config_file").perform(context)
    gnss_config_file = LaunchConfiguration("gnss_config_file").perform(context)
    enable_gnss = LaunchConfiguration("enable_gnss").perform(context).lower() == "true"
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    random_seed = int(LaunchConfiguration("random_seed").perform(context))

    if environment == "legacy":
        world_path = os.path.join(package_share, "worlds", "aruco_moving_deck.sdf")
        model_overrides = {
            "model_name": "moving_deck",
            "initial_position_offset_enu": [0.0, 0.0, 0.0],
            "deck_offset_body": [0.0, 0.0, 0.0],
            "deck_rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "deck_frame_id": "moving_deck",
        }
    else:
        world_path = os.path.join(package_share, "worlds", "aruco_marine_vessel.sdf")
        model_overrides = {
            "model_name": "landing_vessel",
            # 现有 scenario YAML 的 neutral z=2 m 表示 deck；marine 把运动参考点下移到 vessel z=0。
            "initial_position_offset_enu": [0.0, 0.0, -2.0],
            "deck_offset_body": [0.0, 0.0, 2.0],
            "deck_rotation_wxyz": [1.0, 0.0, 0.0, 0.0],
            "deck_frame_id": "landing_deck",
        }

    gz_args = f"-s -r {world_path}" if headless else f"-r --gui-config {gui_config_path} {world_path}"
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={"gz_args": gz_args}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="moving_deck_bridge",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/simulation/deck/ground_truth_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            output="screen",
        ),
        Node(
            package="moving_deck_sim",
            executable="moving_deck_controller",
            name="moving_deck_controller",
            parameters=[
                config_file,
                {"use_sim_time": True, "random_seed": random_seed, **model_overrides},
            ],
            output="screen",
        ),
    ]

    if enable_gnss:
        actions.append(
            Node(
                package="moving_deck_sim",
                executable="deck_gnss_simulator",
                name="deck_gnss_simulator",
                parameters=[
                    gnss_config_file,
                    {"use_sim_time": True, "random_seed": random_seed},
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    package_share = get_package_share_directory("moving_deck_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gui_config_path = os.path.join(package_share, "config", "gazebo_gui.config")
    default_config = os.path.join(package_share, "config", "constant_velocity.yaml")
    default_gnss_config = os.path.join(package_share, "config", "gnss_ideal.yaml")
    gz_launch = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")

    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        [os.path.join(package_share, "models"), existing_resource_path]
    ).rstrip(os.pathsep)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "environment",
                default_value="legacy",
                description="Simulation environment: legacy or marine.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to a moving-deck scenario YAML file.",
            ),
            DeclareLaunchArgument(
                "gnss_config_file",
                default_value=default_gnss_config,
                description="Path to a deck GNSS sensor YAML file.",
            ),
            DeclareLaunchArgument(
                "enable_gnss",
                default_value="true",
                description="Publish simulated deck GNSS position and ENU velocity.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Gazebo server without its GUI.",
            ),
            DeclareLaunchArgument(
                "random_seed",
                default_value="1",
                description="Deterministic seed shared by deck motion and GNSS simulation.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
            OpaqueFunction(
                function=_launch_setup,
                args=[package_share, gz_launch, gui_config_path],
            ),
        ]
    )
