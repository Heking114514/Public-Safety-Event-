from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("visual_navigation")
    default_parameters = PathJoinSubstitution(
        [package_share, "config", "waypoint_navigation.yaml"]
    )
    default_route = PathJoinSubstitution(
        [package_share, "routes", "example_route.csv"]
    )

    parameters_file = LaunchConfiguration("parameters_file")
    route_file = LaunchConfiguration("route_file")
    route_frame = LaunchConfiguration("route_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    fusion_status_topic = LaunchConfiguration("fusion_status_topic")
    actuator_health_topic = LaunchConfiguration("actuator_health_topic")
    require_actuator_health = LaunchConfiguration("require_actuator_health")
    actuator_health_timeout = LaunchConfiguration("actuator_health_timeout")
    tracking_state_topic = LaunchConfiguration("tracking_state_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    route_input_topic = LaunchConfiguration("route_input_topic")
    autostart = LaunchConfiguration("autostart")

    navigator = Node(
        package="visual_navigation",
        executable="waypoint_navigator",
        name="waypoint_navigator",
        output="screen",
        parameters=[
            parameters_file,
            {
                "route_file": ParameterValue(route_file, value_type=str),
                "route_frame": ParameterValue(route_frame, value_type=str),
                "odom_topic": ParameterValue(odom_topic, value_type=str),
                "fusion_status_topic": ParameterValue(
                    fusion_status_topic, value_type=str
                ),
                "actuator_health_topic": ParameterValue(
                    actuator_health_topic, value_type=str
                ),
                "require_actuator_health": ParameterValue(
                    require_actuator_health, value_type=bool
                ),
                "actuator_health_timeout": ParameterValue(
                    actuator_health_timeout, value_type=float
                ),
                "tracking_state_topic": ParameterValue(
                    tracking_state_topic, value_type=str
                ),
                "cmd_vel_topic": ParameterValue(cmd_vel_topic, value_type=str),
                "route_input_topic": ParameterValue(
                    route_input_topic, value_type=str
                ),
                "autostart": ParameterValue(autostart, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="Waypoint navigation parameter file",
            ),
            DeclareLaunchArgument(
                "route_file",
                default_value=default_route,
                description="CSV waypoint route file",
            ),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument(
                "fusion_status_topic", default_value="/odometry/fusion_status"
            ),
            DeclareLaunchArgument(
                "actuator_health_topic", default_value="/cup_car_serial/connected"
            ),
            DeclareLaunchArgument("require_actuator_health", default_value="false"),
            DeclareLaunchArgument("actuator_health_timeout", default_value="0.8"),
            DeclareLaunchArgument(
                "tracking_state_topic", default_value="/tracking_state"
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument(
                "route_input_topic",
                default_value="/waypoint_navigation/route_input",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description="Start only after the localization interface becomes valid",
            ),
            navigator,
        ]
    )
