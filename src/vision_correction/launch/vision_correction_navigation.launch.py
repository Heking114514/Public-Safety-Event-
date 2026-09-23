from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    correction_share = Path(get_package_share_directory("vision_correction"))
    correction_parameters = str(
        correction_share / "config" / "vision_correction.yaml"
    )
    correction = Node(
        package="vision_correction",
        executable="vision_correction_node",
        name="vision_correction_node",
        output="screen",
        parameters=[
            correction_parameters,
            {
                "map_file": LaunchConfiguration("map_file"),
                "output_topic": "/odometry/landmark_corrected",
            },
        ],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("visual_navigation"),
                    "launch",
                    "visual_navigation_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "odom_topic": "/odometry/landmark_corrected",
            "route_file": LaunchConfiguration("route_file"),
            "autostart": LaunchConfiguration("autostart"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument("route_file", default_value=""),
            DeclareLaunchArgument("autostart", default_value="false"),
            correction,
            navigation,
        ]
    )
