from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    loopback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("vehicle_loopback_sim")
            + "/launch/planner_control_loopback.launch.py"
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "autostart": "true",
            "use_waypoint_navigator": "false",
            "use_debug_controller": "false",
            "simulate_obstacle_recognition": "true",
            "enforce_static_collision": "false",
        }.items(),
    )
    ideal = Node(
        package="ideal_controler",
        executable="ideal_controler",
        name="ideal_controler",
        output="screen",
        parameters=[{
            "cmd_vel_topic": "/cmd_vel_nav",
            "target_ids": "1,2,3,4,5,6,7,8,9,10,11,12",
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        loopback,
        ideal,
    ])
