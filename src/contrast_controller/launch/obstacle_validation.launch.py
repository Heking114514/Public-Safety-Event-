"""Contrast 避障闭环：正式任务总控 + 仿真真值检查点授权。"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_config import parameters_for


def generate_launch_description():
    contrast = Path(get_package_share_directory("contrast_controller"))
    manager = Path(get_package_share_directory("navigation_mission_manager"))
    planner = Path(get_package_share_directory("new_map_planner"))
    arena = LaunchConfiguration("planner_config")
    root = "/comparison/contrast"
    controller = root + "/contrast_controller"
    physical = parameters_for("pure_line_following")
    return LaunchDescription([
        DeclareLaunchArgument("planner_config", default_value=str(planner / "config/arena.yaml")),
        DeclareLaunchArgument("target_ids", default_value="1,2,3,4,5,6,7,8,9,10,11,12"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(contrast / "launch/contrast_comparison.launch.py")),
            launch_arguments={
                "controllers": "contrast",
                "planner_config": arena,
                "target_ids": LaunchConfiguration("target_ids"),
                "plan_on_start": "false",
                "obstacle_detection": "true",
                "gui": LaunchConfiguration("gui"),
            }.items()),
        Node(
            package="navigation_mission_manager", executable="mission_manager_node",
            name="navigation_mission_manager", output="screen",
            parameters=[str(manager / "config/mission_manager.yaml"), {
                "map_file": ParameterValue(arena, value_type=str),
                "auto_start": ParameterValue(LaunchConfiguration("auto_start"), value_type=bool),
                "target_ids": ParameterValue(LaunchConfiguration("target_ids"), value_type=str),
                "task_confirmation_mode": "route",
                "odom_topic": root + "/odometry/fused",
                "navigator_status_topic": root + "/navigator_status",
                "navigation_recovery_state_topic": root + "/navigation_recovery_state",
                "current_waypoint_topic": root + "/current_waypoint",
                "route_ack_topic": root + "/route_ack",
                "motion_hold_state_topic": root + "/motion_hold_state",
                "motion_hold_service": controller + "/set_motion_hold",
                "continue_retreat_service": controller + "/continue_obstacle_retreat",
                "continue_route_retreat_service": controller + "/continue_route_retreat",
                "confirm_rfid_checkpoint_service": controller + "/confirm_rfid_checkpoint",
                "select_recovery_service": controller + "/select_recovery",
                "front_obstacle_topic": root + "/obstacle/front_blocked",
                "front_range_topic": root + "/obstacle/front_range",
                "dynamic_obstacle_topic": root + "/obstacle/dynamic",
                "status_text_topic": "/comparison/mission_status",
                "obstacle_sensor_forward_offset_m": physical["camera_x_m"],
            }]),
    ])
