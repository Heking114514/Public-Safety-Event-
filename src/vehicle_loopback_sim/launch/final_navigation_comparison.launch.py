from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_config import parameters_for


def _ros_parameters(path: Path, node_name: str) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    return dict(root[node_name]["ros__parameters"])


def _final_navigation_topics() -> dict[str, str]:
    root = "/comparison/final_navigation"
    return {
        "odom_topic": f"{root}/odometry/fused",
        "imu_topic": f"{root}/imu/control",
        "fusion_status_topic": f"{root}/odometry/fusion_status",
        "actuator_health_topic": f"{root}/actuator/healthy",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "front_obstacle_topic": f"{root}/obstacle/front_blocked",
        "front_obstacle_range_topic": f"{root}/obstacle/front_range",
        "recovery_obstacle_topic": f"{root}/obstacle/dynamic",
        "tracking_state_topic": f"{root}/tracking_state",
        "cmd_vel_topic": f"{root}/cmd_vel",
        "path_topic": f"{root}/waypoint_path",
        "status_topic": f"{root}/navigator_status",
        "current_waypoint_topic": f"{root}/current_waypoint",
        "motion_hold_state_topic": f"{root}/motion_hold_state",
        "route_ack_topic": f"{root}/route_ack",
        "navigation_recovery_state_topic": f"{root}/navigation_recovery_state",
        "start_topic": f"{root}/start",
        "route_input_topic": "/waypoint_navigation/route_input",
    }


def _sim_topics() -> dict[str, str]:
    root = "/comparison/final_navigation"
    return {
        "cmd_vel_topic": f"{root}/cmd_vel",
        "initial_pose_topic": f"{root}/initialpose",
        "odom_topic": f"{root}/odometry/fused",
        "imu_topic": f"{root}/imu/control",
        "fusion_status_topic": f"{root}/odometry/fusion_status",
        "actuator_health_topic": f"{root}/actuator/healthy",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "front_obstacle_topic": f"{root}/obstacle/front_blocked",
        "front_obstacle_range_topic": f"{root}/obstacle/front_range",
        "dynamic_obstacle_topic": f"{root}/obstacle/dynamic",
        "manual_obstacle_topic": f"{root}/obstacle/manual",
        "clear_manual_obstacles_topic": f"{root}/obstacle/clear",
        "status_topic": f"{root}/sim_status",
    }


def _gui_topics() -> dict[str, str]:
    root = "/comparison/final_navigation"
    return {
        "odom_topic": f"{root}/odometry/fused",
        "cmd_vel_topic": f"{root}/cmd_vel",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "loopback_status_topic": f"{root}/sim_status",
        "navigator_status_topic": f"{root}/navigator_status",
        "current_waypoint_topic": f"{root}/current_waypoint",
        "waypoint_path_topic": f"{root}/waypoint_path",
        "manual_obstacle_topic": f"{root}/obstacle/manual",
        "clear_manual_obstacles_topic": f"{root}/obstacle/clear",
        "navigation_path_topic": "/new_map_planner/navigation_path",
        "arena_path_topic": "/new_map_planner/arena_path",
        "map_topic": "/new_map_planner/map",
        "mission_status_topic": "/comparison/mission_status",
    }


def generate_launch_description():
    planner_share = Path(get_package_share_directory("new_map_planner"))
    final_share = Path(get_package_share_directory("final_navigation"))
    simulator_share = Path(get_package_share_directory("vehicle_loopback_sim"))
    vehicle_share = Path(get_package_share_directory("vehicle_config"))
    default_arena = str(planner_share / "config" / "arena.yaml")
    vehicle_file = str(vehicle_share / "config" / "vehicle_physical.yaml")
    final_config = final_share / "config" / "final_navigation.yaml"
    loopback_config = simulator_share / "config" / "vehicle_loopback.yaml"
    final_parameters = _ros_parameters(final_config, "final_navigation")
    simulation_parameters = _ros_parameters(
        final_share / "config" / "final_navigation_simulation_turn.yaml",
        "final_navigation",
    )
    sim_parameters = _ros_parameters(loopback_config, "vehicle_loopback_sim")
    gui_parameters = _ros_parameters(loopback_config, "vehicle_loopback_gui")
    mission_parameters = _ros_parameters(loopback_config, "vehicle_loopback_mission")
    vehicle_parameters = parameters_for("waypoint_navigation")
    simulator_physical = parameters_for("pure_line_following")

    planner_config = LaunchConfiguration("planner_config")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    gui = LaunchConfiguration("gui")
    gui_host = LaunchConfiguration("gui_host")
    plan_on_start = LaunchConfiguration("plan_on_start")
    planner_mode = LaunchConfiguration("planner_mode")
    target_ids = LaunchConfiguration("target_ids")
    return_home = LaunchConfiguration("return_home")
    plan_request_delay = LaunchConfiguration("plan_request_delay")
    simulation_time_scale = LaunchConfiguration("simulation_time_scale")
    use_sim_time = True

    clock = Node(
        package="vehicle_loopback_sim",
        executable="simulation_clock_node",
        name="simulation_clock",
        output="screen",
        parameters=[{"simulation_time_scale": ParameterValue(simulation_time_scale, value_type=float)}],
    )
    planner = Node(
        package="new_map_planner",
        executable="new_map_planner_node",
        name="new_map_planner",
        output="screen",
        parameters=[
            {"map_file": ParameterValue(planner_config, value_type=str),
             "vehicle_file": vehicle_file, "use_sim_time": use_sim_time}
        ],
    )
    final_navigator = Node(
        package="final_navigation",
        executable="final_navigation",
        namespace="comparison/final_navigation",
        name="final_navigation",
        output="screen",
        parameters=[
            final_parameters,
            vehicle_parameters,
            _final_navigation_topics(),
            {"recovery_map_file": ParameterValue(planner_config, value_type=str),
             **simulation_parameters,
             "use_sim_time": use_sim_time,
             "control_rate_multiplier": 1.0},
        ],
    )
    shared_sim_parameters = {
        "arena_config": ParameterValue(planner_config, value_type=str),
        "initial_x_m": ParameterValue(initial_x, value_type=float),
        "initial_y_m": ParameterValue(initial_y, value_type=float),
        "initial_yaw_rad": ParameterValue(initial_yaw, value_type=float),
        "publish_tf": False,
        "enforce_static_collision": True,
        "publish_obstacle_detection": ParameterValue(
            LaunchConfiguration("obstacle_detection"), value_type=bool),
        "wheel_track_m": simulator_physical["wheel_track_m"],
        "base_offset_x_m": simulator_physical["base_offset_x_m"],
        "base_offset_y_m": simulator_physical["base_offset_y_m"],
        "playback_speed": 1.0,
        "use_sim_time": use_sim_time,
        "obstacle_sensor_x_m": simulator_physical["camera_x_m"],
        "obstacle_sensor_y_m": simulator_physical["camera_y_m"],
    }
    simulator = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_sim_node",
        namespace="comparison/final_navigation",
        name="vehicle_loopback_sim",
        output="screen",
        parameters=[sim_parameters, _sim_topics(), shared_sim_parameters],
    )
    mission = Node(
        package="vehicle_loopback_sim",
        executable="final_navigation_mission_node",
        namespace="comparison",
        name="vehicle_loopback_mission",
        output="screen",
        condition=IfCondition(plan_on_start),
        parameters=[
            mission_parameters,
            {
                "use_sim_time": use_sim_time,
                "external_mission_manager": ParameterValue(
                    LaunchConfiguration("external_mission_manager"), value_type=bool),
                "arena_config": ParameterValue(planner_config, value_type=str),
                "mode": ParameterValue(planner_mode, value_type=str),
                "target_ids": ParameterValue(target_ids, value_type=str),
                "return_home": ParameterValue(return_home, value_type=bool),
                "allow_partial": True,
                "planner_service": "/new_map_planner/plan",
                "request_delay_s": ParameterValue(plan_request_delay, value_type=float),
                "status_topic": "/comparison/mission_status",
                "navigator_status_topic": "/comparison/final_navigation/navigator_status",
                "navigator_status_topics": "/comparison/final_navigation/navigator_status",
                "odom_topic": "/comparison/final_navigation/odometry/fused",
            },
        ],
    )
    gui_node = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_gui_node",
        namespace="comparison/final_navigation",
        name="vehicle_loopback_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[
            gui_parameters,
            _gui_topics(),
            {"display_name": "Final", "host": ParameterValue(gui_host, value_type=str),
             "port": 8765, "arena_config": ParameterValue(planner_config, value_type=str),
             "use_sim_time": use_sim_time},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("planner_config", default_value=default_arena),
        DeclareLaunchArgument("planner_mode", default_value=str(mission_parameters["mode"])),
        DeclareLaunchArgument("target_ids", default_value=str(mission_parameters["target_ids"])),
        DeclareLaunchArgument("return_home", default_value=str(mission_parameters["return_home"]).lower()),
        DeclareLaunchArgument("plan_request_delay", default_value=str(mission_parameters["request_delay_s"])),
        DeclareLaunchArgument("simulation_time_scale", default_value="1.0"),
        DeclareLaunchArgument("initial_x", default_value=str(sim_parameters["initial_x_m"])),
        DeclareLaunchArgument("initial_y", default_value=str(sim_parameters["initial_y_m"])),
        DeclareLaunchArgument("initial_yaw", default_value=str(sim_parameters["initial_yaw_rad"])),
        DeclareLaunchArgument("plan_on_start", default_value="true"),
        DeclareLaunchArgument("external_mission_manager", default_value="false"),
        DeclareLaunchArgument("obstacle_detection", default_value="false"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("gui_host", default_value=str(gui_parameters["host"])),
        clock, planner, final_navigator, simulator, mission, gui_node,
    ])
