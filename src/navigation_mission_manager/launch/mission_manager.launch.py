from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("navigation_mission_manager"))
    planner_share = Path(get_package_share_directory("new_map_planner"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file", default_value=str(planner_share / "config" / "arena.yaml")
            ),
            DeclareLaunchArgument("auto_start", default_value="false"),
            DeclareLaunchArgument(
                "status_text_topic", default_value="/navigation_mission/status_text"
            ),
            DeclareLaunchArgument(
                "dynamic_obstacle_topic",
                default_value="/perception/dynamic_obstacle",
            ),
            Node(
                package="navigation_mission_manager",
                executable="mission_manager_node",
                name="navigation_mission_manager",
                output="screen",
                parameters=[
                    str(package_share / "config" / "mission_manager.yaml"),
                    {
                        "map_file": ParameterValue(
                            LaunchConfiguration("map_file"), value_type=str
                        ),
                        "auto_start": ParameterValue(
                            LaunchConfiguration("auto_start"), value_type=bool
                        ),
                        "status_text_topic": ParameterValue(
                            LaunchConfiguration("status_text_topic"), value_type=str
                        ),
                        "dynamic_obstacle_topic": ParameterValue(
                            LaunchConfiguration("dynamic_obstacle_topic"),
                            value_type=str,
                        ),
                    },
                ],
            ),
        ]
    )
