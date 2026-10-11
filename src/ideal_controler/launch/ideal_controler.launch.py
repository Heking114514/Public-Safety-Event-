from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("route_input_topic", default_value="/waypoint_navigation/route_input"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_ideal"),
        Node(
            package="ideal_controler",
            executable="ideal_controler",
            name="ideal_controler",
            output="screen",
            parameters=[{
                "route_input_topic": LaunchConfiguration("route_input_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }],
        ),
    ])
