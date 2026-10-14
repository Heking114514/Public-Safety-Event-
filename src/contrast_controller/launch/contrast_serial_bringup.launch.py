from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from vehicle_config import parameters_for


def generate_launch_description():
    visual_share = Path(get_package_share_directory("visual_navigation"))
    contrast_share_path = Path(get_package_share_directory("contrast_controller"))
    map_share = Path(get_package_share_directory("new_map_planner"))
    serial_share = FindPackageShare("cup_car_serial")
    contrast_share = FindPackageShare("contrast_controller")
    production_config = visual_share / "config" / "waypoint_navigation.yaml"
    contrast_config = contrast_share_path / "config" / "contrast_controller.yaml"
    default_recovery_map = str(map_share / "config" / "arena.yaml")
    with contrast_config.open("r", encoding="utf-8") as stream:
        contrast_parameters = yaml.safe_load(stream)["contrast_controller"][
            "ros__parameters"
        ]
    physical_parameters = parameters_for("waypoint_navigation")

    parameters_file = LaunchConfiguration("parameters_file")
    route_file = LaunchConfiguration("route_file")
    route_frame = LaunchConfiguration("route_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    fusion_status_topic = LaunchConfiguration("fusion_status_topic")
    actuator_health_topic = LaunchConfiguration("actuator_health_topic")
    actuator_health_timeout = LaunchConfiguration("actuator_health_timeout")
    front_obstacle_topic = LaunchConfiguration("front_obstacle_topic")
    front_obstacle_range_topic = LaunchConfiguration("front_obstacle_range_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    route_input_topic = LaunchConfiguration("route_input_topic")
    recovery_map_file = LaunchConfiguration("recovery_map_file")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    serial_send_rate_hz = LaunchConfiguration("serial_send_rate_hz")
    serial_command_timeout_s = LaunchConfiguration("serial_command_timeout_s")
    serial_control_telemetry_timeout_s = LaunchConfiguration(
        "serial_control_telemetry_timeout_s"
    )

    navigator = Node(
        package="contrast_controller",
        executable="contrast_controller",
        name="waypoint_navigator",
        output="screen",
        parameters=[
            parameters_file,
            contrast_parameters,
            physical_parameters,
            {
                "route_file": ParameterValue(route_file, value_type=str),
                "route_frame": ParameterValue(route_frame, value_type=str),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
                "tracking_point_offset_x": physical_parameters[
                    "tracking_point_offset_x"
                ],
                "tracking_point_offset_y": physical_parameters[
                    "tracking_point_offset_y"
                ],
                "fusion_status_topic": ParameterValue(
                    fusion_status_topic, value_type=str
                ),
                "actuator_health_topic": ParameterValue(
                    actuator_health_topic, value_type=str
                ),
                "require_actuator_health": True,
                "actuator_health_timeout": ParameterValue(
                    actuator_health_timeout, value_type=float
                ),
                "front_obstacle_topic": ParameterValue(
                    front_obstacle_topic, value_type=str
                ),
                "front_obstacle_range_topic": ParameterValue(
                    front_obstacle_range_topic, value_type=str
                ),
                "cmd_vel_topic": ParameterValue(cmd_vel_topic, value_type=str),
                "route_input_topic": ParameterValue(
                    route_input_topic, value_type=str
                ),
                "recovery_map_file": ParameterValue(
                    recovery_map_file, value_type=str
                ),
                "autostart": ParameterValue(autostart, value_type=bool),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    serial_bridge = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [serial_share, "config", "cmd_vel_serial.yaml"]
            ),
            {
                "device": ParameterValue(serial_device, value_type=str),
                "baud_rate": ParameterValue(serial_baud_rate, value_type=int),
                "topic": ParameterValue(cmd_vel_topic, value_type=str),
                "send_rate_hz": ParameterValue(
                    serial_send_rate_hz, value_type=float
                ),
                "command_timeout_s": ParameterValue(
                    serial_command_timeout_s, value_type=float
                ),
                "control_telemetry_timeout_s": ParameterValue(
                    serial_control_telemetry_timeout_s, value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file", default_value=str(production_config)
            ),
            DeclareLaunchArgument("route_file", default_value=""),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument(
                "fusion_status_topic", default_value="/odometry/fusion_status"
            ),
            DeclareLaunchArgument(
                "actuator_health_topic",
                default_value="/cup_car_serial/actuator_healthy",
            ),
            DeclareLaunchArgument("actuator_health_timeout", default_value="0.8"),
            DeclareLaunchArgument(
                "front_obstacle_topic", default_value="/obstacle/front_blocked"
            ),
            DeclareLaunchArgument(
                "front_obstacle_range_topic", default_value="/obstacle/front_range"
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument(
                "route_input_topic",
                default_value="/waypoint_navigation/route_input",
            ),
            DeclareLaunchArgument(
                "recovery_map_file", default_value=default_recovery_map
            ),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("serial_device", default_value="auto"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("serial_send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("serial_command_timeout_s", default_value="0.4"),
            DeclareLaunchArgument(
                "serial_control_telemetry_timeout_s", default_value="0.90"
            ),
            navigator,
            serial_bridge,
        ]
    )
