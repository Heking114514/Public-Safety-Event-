from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_navigation")
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
    autostart = LaunchConfiguration("autostart")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud_rate = LaunchConfiguration("serial_baud_rate")
    serial_send_rate_hz = LaunchConfiguration("serial_send_rate_hz")
    serial_command_timeout_s = LaunchConfiguration("serial_command_timeout_s")
    serial_control_telemetry_timeout_s = LaunchConfiguration(
        "serial_control_telemetry_timeout_s"
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "visual_navigation_bringup.launch.py"]
            )
        ),
        launch_arguments={
            "route_file": route_file,
            "route_frame": route_frame,
            "odom_topic": odom_topic,
            "fusion_status_topic": fusion_status_topic,
            "actuator_health_topic": actuator_health_topic,
            "require_actuator_health": "true",
            "actuator_health_timeout": actuator_health_timeout,
            "front_obstacle_topic": front_obstacle_topic,
            "front_obstacle_range_topic": front_obstacle_range_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "route_input_topic": route_input_topic,
            "autostart": autostart,
        }.items(),
    )
    serial_bridge = Node(
        package="cup_car_serial",
        executable="cmd_vel_serial_node",
        name="cmd_vel_serial_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("cup_car_serial"),
                    "config",
                    "cmd_vel_serial.yaml",
                ]
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
            }
        ],
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("serial_device", default_value="auto"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("serial_send_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("serial_command_timeout_s", default_value="0.4"),
            DeclareLaunchArgument(
                "serial_control_telemetry_timeout_s", default_value="0.90"
            ),
            navigation,
            serial_bridge,
        ]
    )
