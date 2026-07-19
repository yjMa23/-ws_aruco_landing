import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("moving_deck_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    world_path = os.path.join(package_share, "worlds", "aruco_moving_deck.sdf")
    default_config = os.path.join(
        package_share, "config", "constant_velocity.yaml"
    )
    gz_launch = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")

    config_file = LaunchConfiguration("config_file")
    headless = LaunchConfiguration("headless")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        [os.path.join(package_share, "models"), existing_resource_path]
    ).rstrip(os.pathsep)

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-s -r {world_path}"}.items(),
        condition=IfCondition(headless),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="moving_deck_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/simulation/deck/ground_truth_raw@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        output="screen",
    )
    controller = Node(
        package="moving_deck_sim",
        executable="moving_deck_controller",
        name="moving_deck_controller",
        parameters=[config_file, {"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to a moving-deck scenario YAML file.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run the Gazebo server without its GUI.",
            ),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            # ponytail: P1 仅支持同主机 PX4；需要远程 SITL 时再开放地址参数。
            SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
            gazebo_gui,
            gazebo_headless,
            bridge,
            controller,
        ]
    )
