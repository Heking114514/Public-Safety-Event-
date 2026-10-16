from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_config import parameters_for


def generate_launch_description():
    share = Path(get_package_share_directory("final_navigation"))
    planner = Path(get_package_share_directory("new_map_planner"))
    config = share / "config" / "final_navigation.yaml"
    with config.open("r", encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["final_navigation"]["ros__parameters"]
    physical = parameters_for("waypoint_navigation")
    return LaunchDescription([
        DeclareLaunchArgument("parameters_file", default_value=str(config)),
        DeclareLaunchArgument("route_file", default_value=""),
        DeclareLaunchArgument("route_frame", default_value="map"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
        DeclareLaunchArgument("fusion_status_topic", default_value="/odometry/fusion_status"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
        DeclareLaunchArgument("route_input_topic", default_value="/waypoint_navigation/route_input"),
        DeclareLaunchArgument("recovery_map_file", default_value=str(planner / "config" / "arena.yaml")),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("control_rate_multiplier", default_value="1.0"),
        Node(
            package="final_navigation",
            executable="final_navigation",
            name="final_navigation",
            output="screen",
            parameters=[
                LaunchConfiguration("parameters_file"),
                parameters,
                physical,
                {
                    "route_file": ParameterValue(LaunchConfiguration("route_file"), value_type=str),
                    "route_frame": ParameterValue(LaunchConfiguration("route_frame"), value_type=str),
                    "odom_topic": ParameterValue(LaunchConfiguration("odom_topic"), value_type=str),
                    "fusion_status_topic": ParameterValue(LaunchConfiguration("fusion_status_topic"), value_type=str),
                    "cmd_vel_topic": ParameterValue(LaunchConfiguration("cmd_vel_topic"), value_type=str),
                    "route_input_topic": ParameterValue(LaunchConfiguration("route_input_topic"), value_type=str),
                    "recovery_map_file": ParameterValue(LaunchConfiguration("recovery_map_file"), value_type=str),
                    "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
