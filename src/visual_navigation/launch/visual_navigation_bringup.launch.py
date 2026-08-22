from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_navigation")
    route_file = LaunchConfiguration("route_file")
    route_frame = LaunchConfiguration("route_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    fusion_status_topic = LaunchConfiguration("fusion_status_topic")
    actuator_health_topic = LaunchConfiguration("actuator_health_topic")
    require_actuator_health = LaunchConfiguration("require_actuator_health")
    actuator_health_timeout = LaunchConfiguration("actuator_health_timeout")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    route_input_topic = LaunchConfiguration("route_input_topic")
    autostart = LaunchConfiguration("autostart")

    waypoint_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "waypoint_navigation.launch.py"]
            )
        ),
        launch_arguments={
            "route_file": route_file,
            "route_frame": route_frame,
            "odom_topic": odom_topic,
            "fusion_status_topic": fusion_status_topic,
            "actuator_health_topic": actuator_health_topic,
            "require_actuator_health": require_actuator_health,
            "actuator_health_timeout": actuator_health_timeout,
            "cmd_vel_topic": cmd_vel_topic,
            "route_input_topic": route_input_topic,
            "autostart": autostart,
        }.items(),
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
            DeclareLaunchArgument("require_actuator_health", default_value="false"),
            DeclareLaunchArgument("actuator_health_timeout", default_value="0.8"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument(
                "route_input_topic",
                default_value="/waypoint_navigation/route_input",
            ),
            DeclareLaunchArgument("autostart", default_value="false"),
            waypoint_navigation,
        ]
    )
