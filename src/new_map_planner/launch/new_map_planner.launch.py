from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_ui = LaunchConfiguration("start_ui")
    ui = Node(
        package="new_map_planner",
        executable="new_map_planner_ui",
        name="new_map_planner_ui",
        output="screen",
        condition=IfCondition(start_ui),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_ui", default_value="true"),
            DeclareLaunchArgument(
                "map_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("new_map_planner"), "config", "arena.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "vehicle_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("vehicle_config"),
                        "config",
                        "vehicle_physical.yaml",
                    ]
                ),
            ),
            Node(
                package="new_map_planner",
                executable="new_map_planner_node",
                name="new_map_planner",
                output="screen",
                parameters=[
                    {
                        "map_file": LaunchConfiguration("map_file"),
                        "vehicle_file": LaunchConfiguration("vehicle_file"),
                    }
                ],
            ),
            ui,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ui,
                    on_exit=[EmitEvent(event=Shutdown(reason="operator UI closed"))],
                ),
                condition=IfCondition(start_ui),
            ),
        ]
    )
