from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    manager_share = Path(get_package_share_directory("navigation_mission_manager"))
    planner_share = Path(get_package_share_directory("new_map_planner"))
    vehicle_share = Path(get_package_share_directory("vehicle_config"))
    map_file = LaunchConfiguration("map_file")
    vehicle_file = LaunchConfiguration("vehicle_file")
    start_ui = LaunchConfiguration("start_ui")

    manager_launch = manager_share / "launch" / "mission_manager.launch.py"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file", default_value=str(planner_share / "config" / "arena.yaml")
            ),
            DeclareLaunchArgument(
                "vehicle_file",
                default_value=str(vehicle_share / "config" / "vehicle_physical.yaml"),
            ),
            DeclareLaunchArgument("start_ui", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            Node(
                package="new_map_planner",
                executable="new_map_planner_node",
                name="new_map_planner",
                output="screen",
                parameters=[
                    {
                        "map_file": ParameterValue(map_file, value_type=str),
                        "vehicle_file": ParameterValue(vehicle_file, value_type=str),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(manager_launch)),
                launch_arguments={
                    "map_file": map_file,
                    "auto_start": LaunchConfiguration("auto_start"),
                }.items(),
            ),
            Node(
                package="new_map_planner",
                executable="new_map_planner_ui",
                name="new_map_planner_ui",
                output="screen",
                condition=IfCondition(start_ui),
            ),
        ]
    )
