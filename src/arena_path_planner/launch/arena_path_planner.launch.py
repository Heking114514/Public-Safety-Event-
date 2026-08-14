from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("arena_path_planner"))
        / "config"
        / "arena_map.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="arena_path_planner",
                executable="arena_planner_node",
                name="arena_path_planner",
                output="screen",
                parameters=[{"config_file": LaunchConfiguration("config")}],
            ),
        ]
    )
