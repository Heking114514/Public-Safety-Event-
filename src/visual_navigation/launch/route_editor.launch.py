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
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument(
                "route_feedback_topic", default_value="/waypoint_path"
            ),
            DeclareLaunchArgument("odom_timeout", default_value="0.5"),
            DeclareLaunchArgument("activation_timeout", default_value="3.0"),
            Node(
                package="visual_navigation",
                executable="route_editor.py",
                name="waypoint_route_editor",
                output="screen",
                parameters=[
                    {
                        "odom_topic": ParameterValue(
                            LaunchConfiguration("odom_topic"), value_type=str
                        ),
                        "route_input_topic": ParameterValue(
                            LaunchConfiguration("route_input_topic"), value_type=str
                        ),
                        "route_frame": ParameterValue(
                            LaunchConfiguration("route_frame"), value_type=str
                        ),
                        "route_feedback_topic": ParameterValue(
                            LaunchConfiguration("route_feedback_topic"), value_type=str
                        ),
                        "odom_timeout": ParameterValue(
                            LaunchConfiguration("odom_timeout"), value_type=float
                        ),
                        "activation_timeout": ParameterValue(
                            LaunchConfiguration("activation_timeout"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
