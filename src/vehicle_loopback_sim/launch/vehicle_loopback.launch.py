from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from vehicle_config import parameters_for


def generate_launch_description():
    package_share = FindPackageShare("vehicle_loopback_sim")
    default_parameters = PathJoinSubstitution(
        [package_share, "config", "vehicle_loopback.yaml"]
    )
    default_arena_config = PathJoinSubstitution(
        [FindPackageShare("new_map_planner"), "config", "arena.yaml"]
    )
    parameters_file = LaunchConfiguration("parameters_file")
    arena_config = LaunchConfiguration("arena_config")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    publish_tf = LaunchConfiguration("publish_tf")
    gui = LaunchConfiguration("gui")
    gui_host = LaunchConfiguration("gui_host")
    gui_port = LaunchConfiguration("gui_port")
    simulate_obstacle_recognition = LaunchConfiguration(
        "simulate_obstacle_recognition"
    )
    physical = parameters_for("pure_line_following")

    loopback = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_sim_node",
        name="vehicle_loopback_sim",
        output="screen",
        parameters=[
            parameters_file,
            {
                "cmd_vel_topic": ParameterValue(cmd_vel_topic, value_type=str),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
                "arena_config": ParameterValue(arena_config, value_type=str),
                "initial_x_m": ParameterValue(initial_x, value_type=float),
                "initial_y_m": ParameterValue(initial_y, value_type=float),
                "initial_yaw_rad": ParameterValue(initial_yaw, value_type=float),
                "publish_tf": ParameterValue(publish_tf, value_type=bool),
                "wheel_track_m": physical["wheel_track_m"],
                "base_offset_x_m": physical["base_offset_x_m"],
                "base_offset_y_m": physical["base_offset_y_m"],
                "publish_obstacle_detection": ParameterValue(
                    simulate_obstacle_recognition, value_type=bool
                ),
            },
        ],
    )

    gui_node = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_gui_node",
        name="vehicle_loopback_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[
            parameters_file,
            {
                "host": ParameterValue(gui_host, value_type=str),
                "port": ParameterValue(gui_port, value_type=int),
                "arena_config": ParameterValue(arena_config, value_type=str),
                "cmd_vel_topic": ParameterValue(cmd_vel_topic, value_type=str),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="Vehicle loopback parameter file",
            ),
            DeclareLaunchArgument(
                "arena_config",
                default_value=default_arena_config,
                description="Arena map YAML used by the GUI layout",
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument(
                "simulate_obstacle_recognition",
                default_value="true",
                description="Publish simulated front obstacle reports from the map",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the browser visualization node",
            ),
            DeclareLaunchArgument("gui_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("gui_port", default_value="8765"),
            loopback,
            gui_node,
        ]
    )
