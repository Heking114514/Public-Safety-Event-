from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_share = Path(get_package_share_directory("new_map_planner"))
    loopback_share = Path(get_package_share_directory("vehicle_loopback_sim"))
    mission_share = Path(get_package_share_directory("navigation_mission_manager"))
    planner_config = LaunchConfiguration("planner_config")
    target_ids = LaunchConfiguration("target_ids")
    gui = LaunchConfiguration("gui")

    loopback = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(loopback_share / "launch" / "vehicle_loopback.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "arena_config": planner_config,
            "cmd_vel_topic": "/cmd_vel_nav",
            "odom_topic": "/odometry/fused",
            "simulate_obstacle_recognition": "true",
            "enforce_static_collision": "false",
        }.items(),
    )
    planner = Node(
        package="new_map_planner",
        executable="new_map_planner_node",
        name="new_map_planner",
        output="screen",
        parameters=[{"map_file": ParameterValue(planner_config, value_type=str)}],
    )
    mission = Node(
        package="navigation_mission_manager",
        executable="mission_manager_node",
        name="navigation_mission_manager",
        output="screen",
        parameters=[
            str(mission_share / "config" / "mission_manager.yaml"),
            {
                "map_file": ParameterValue(planner_config, value_type=str),
                "auto_start": True,
                "target_ids": ParameterValue(target_ids, value_type=str),
                "planner_service": "/new_map_planner/plan",
                "odom_topic": "/odometry/fused",
                "navigator_status_topic": "/waypoint_navigation/status",
                "route_ack_topic": "/waypoint_navigation/route_ack",
                "task_confirmation_mode": "route",
                "motion_hold_service": "/final_navigation/set_motion_hold",
                "motion_hold_state_topic": "/waypoint_navigation/motion_hold_state",
                "front_obstacle_topic": "/obstacle/front_blocked",
                "front_range_topic": "/obstacle/front_range",
                "status_text_topic": "/vehicle_loopback_sim/mission_status",
            },
        ],
    )
    ideal = Node(
        package="ideal_controler",
        executable="ideal_controler",
        name="ideal_controler",
        output="screen",
        parameters=[{"cmd_vel_topic": "/cmd_vel_nav"}],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "planner_config", default_value=str(planner_share / "config" / "arena.yaml")
        ),
        DeclareLaunchArgument(
            "target_ids", default_value="1,2,3,4,5,6,7,8,9,10,11,12"
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        loopback,
        planner,
        ideal,
        mission,
    ])
