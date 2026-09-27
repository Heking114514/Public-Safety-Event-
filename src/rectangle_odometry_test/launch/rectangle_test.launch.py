from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument(
                "route_input_topic",
                default_value="/waypoint_navigation/route_input",
            ),
            DeclareLaunchArgument(
                "route_ack_topic",
                default_value="/waypoint_navigation/route_ack",
            ),
            DeclareLaunchArgument(
                "status_topic",
                default_value="/waypoint_navigation/status",
            ),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("child_frame", default_value="base_link"),
            DeclareLaunchArgument("side_length_m", default_value="1.8"),
            DeclareLaunchArgument("corner_mode", default_value="pivot"),
            DeclareLaunchArgument("corner_radius_m", default_value="0.25"),
            DeclareLaunchArgument("corner_samples", default_value="16"),
            DeclareLaunchArgument("straight_step_m", default_value="0.05"),
            DeclareLaunchArgument("startup_delay_s", default_value="1.0"),
            DeclareLaunchArgument("publish_retry_period_s", default_value="0.5"),
            DeclareLaunchArgument("anchor_to_current_pose", default_value="true"),
            DeclareLaunchArgument("start_x", default_value="0.0"),
            DeclareLaunchArgument("start_y", default_value="0.0"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            Node(
                package="rectangle_odometry_test",
                executable="rectangle_route_node",
                name="rectangle_odometry_test",
                output="screen",
                parameters=[
                    {
                        "odom_topic": ParameterValue(
                            LaunchConfiguration("odom_topic"), value_type=str
                        ),
                        "route_input_topic": ParameterValue(
                            LaunchConfiguration("route_input_topic"), value_type=str
                        ),
                        "route_ack_topic": ParameterValue(
                            LaunchConfiguration("route_ack_topic"), value_type=str
                        ),
                        "status_topic": ParameterValue(
                            LaunchConfiguration("status_topic"), value_type=str
                        ),
                        "route_frame": ParameterValue(
                            LaunchConfiguration("route_frame"), value_type=str
                        ),
                        "child_frame": ParameterValue(
                            LaunchConfiguration("child_frame"), value_type=str
                        ),
                        "side_length_m": ParameterValue(
                            LaunchConfiguration("side_length_m"), value_type=float
                        ),
                        "corner_mode": ParameterValue(
                            LaunchConfiguration("corner_mode"), value_type=str
                        ),
                        "corner_radius_m": ParameterValue(
                            LaunchConfiguration("corner_radius_m"), value_type=float
                        ),
                        "corner_samples": ParameterValue(
                            LaunchConfiguration("corner_samples"), value_type=int
                        ),
                        "straight_step_m": ParameterValue(
                            LaunchConfiguration("straight_step_m"), value_type=float
                        ),
                        "startup_delay_s": ParameterValue(
                            LaunchConfiguration("startup_delay_s"), value_type=float
                        ),
                        "publish_retry_period_s": ParameterValue(
                            LaunchConfiguration(
                                "publish_retry_period_s"
                            ),
                            value_type=float,
                        ),
                        "anchor_to_current_pose": ParameterValue(
                            LaunchConfiguration("anchor_to_current_pose"),
                            value_type=bool,
                        ),
                        "start_x": ParameterValue(
                            LaunchConfiguration("start_x"), value_type=float
                        ),
                        "start_y": ParameterValue(
                            LaunchConfiguration("start_y"), value_type=float
                        ),
                        "start_yaw": ParameterValue(
                            LaunchConfiguration("start_yaw"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
