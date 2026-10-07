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


def _controller_topics(controller: str) -> dict[str, str]:
    root = f"/comparison/{controller}"
    return {
        "odom_topic": f"{root}/odometry/fused",
        "imu_topic": f"{root}/imu/control",
        "fusion_status_topic": f"{root}/odometry/fusion_status",
        "actuator_health_topic": f"{root}/actuator/healthy",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "front_obstacle_topic": f"{root}/obstacle/front_blocked",
        "front_obstacle_range_topic": f"{root}/obstacle/front_range",
        "tracking_state_topic": f"{root}/tracking_state",
        "cmd_vel_topic": f"{root}/cmd_vel",
        "path_topic": f"{root}/waypoint_path",
        "status_topic": f"{root}/navigator_status",
        "current_waypoint_topic": f"{root}/current_waypoint",
        "motion_hold_state_topic": f"{root}/motion_hold_state",
        "route_ack_topic": f"{root}/route_ack",
        "start_topic": f"{root}/start",
        "route_input_topic": "/waypoint_navigation/route_input",
    }


def _sim_topics(controller: str) -> dict[str, str]:
    root = f"/comparison/{controller}"
    return {
        "cmd_vel_topic": f"{root}/cmd_vel",
        "initial_pose_topic": f"{root}/initialpose",
        "odom_topic": f"{root}/odometry/fused",
        "imu_topic": f"{root}/imu/control",
        "fusion_status_topic": f"{root}/odometry/fusion_status",
        "actuator_health_topic": f"{root}/actuator/healthy",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "status_topic": f"{root}/sim_status",
    }


def _gui_topics(controller: str) -> dict[str, str]:
    root = f"/comparison/{controller}"
    return {
        "odom_topic": f"{root}/odometry/fused",
        "cmd_vel_topic": f"{root}/cmd_vel",
        "control_telemetry_topic": f"{root}/control_telemetry",
        "loopback_status_topic": f"{root}/sim_status",
        "navigator_status_topic": f"{root}/navigator_status",
        "current_waypoint_topic": f"{root}/current_waypoint",
        "waypoint_path_topic": f"{root}/waypoint_path",
        "navigation_path_topic": "/new_map_planner/navigation_path",
        "arena_path_topic": "/new_map_planner/arena_path",
        "map_topic": "/new_map_planner/map",
        "mission_status_topic": "/comparison/mission_status",
    }


def generate_launch_description():
    planner_share = Path(get_package_share_directory("new_map_planner"))
    navigation_share = Path(get_package_share_directory("visual_navigation"))
    simulator_share = Path(get_package_share_directory("vehicle_loopback_sim"))

    default_arena = str(planner_share / "config" / "arena.yaml")
    new_config = navigation_share / "config" / "waypoint_navigation.yaml"
    old_config = navigation_share / "config" / "waypoint_navigation_legacy.yaml"
    loopback_config = simulator_share / "config" / "vehicle_loopback.yaml"

    new_parameters = _ros_parameters(new_config, "waypoint_navigator")
    old_parameters = _ros_parameters(old_config, "waypoint_navigator")
    sim_parameters = _ros_parameters(loopback_config, "vehicle_loopback_sim")
    gui_parameters = _ros_parameters(loopback_config, "vehicle_loopback_gui")
    mission_parameters = _ros_parameters(loopback_config, "vehicle_loopback_mission")
    navigation_physical = parameters_for("waypoint_navigation")
    legacy_physical = {
        key: navigation_physical[key]
        for key in (
            "tracking_point_offset_x",
            "tracking_point_offset_y",
            "obstacle_sensor_forward_offset",
            "vehicle_front_offset",
        )
    }
    simulator_physical = parameters_for("pure_line_following")

    arena_config = LaunchConfiguration("planner_config")
    gui = LaunchConfiguration("gui")
    new_gui_port = LaunchConfiguration("new_gui_port")
    old_gui_port = LaunchConfiguration("old_gui_port")
    plan_on_start = LaunchConfiguration("plan_on_start")
    planner_mode = LaunchConfiguration("planner_mode")
    target_ids = LaunchConfiguration("target_ids")
    return_home = LaunchConfiguration("return_home")
    plan_request_delay = LaunchConfiguration("plan_request_delay")

    planner = Node(
        package="new_map_planner",
        executable="new_map_planner_node",
        name="new_map_planner",
        output="screen",
        parameters=[{"map_file": ParameterValue(arena_config, value_type=str)}],
    )

    new_navigator = Node(
        package="visual_navigation",
        executable="waypoint_navigator",
        namespace="comparison/new",
        name="waypoint_navigator",
        output="screen",
        parameters=[
            new_parameters,
            navigation_physical,
            _controller_topics("new"),
        ],
    )
    old_navigator = Node(
        package="visual_navigation",
        executable="waypoint_navigator_legacy",
        namespace="comparison/old",
        name="waypoint_navigator",
        output="screen",
        parameters=[
            old_parameters,
            legacy_physical,
            _controller_topics("old"),
        ],
    )

    simulators = []
    for controller in ("new", "old"):
        simulators.append(
            Node(
                package="vehicle_loopback_sim",
                executable="vehicle_loopback_sim_node",
                namespace=f"comparison/{controller}",
                name="vehicle_loopback_sim",
                output="screen",
                parameters=[
                    sim_parameters,
                    _sim_topics(controller),
                    {
                        "publish_tf": False,
                        "wheel_track_m": simulator_physical["wheel_track_m"],
                        "base_offset_x_m": simulator_physical["base_offset_x_m"],
                        "base_offset_y_m": simulator_physical["base_offset_y_m"],
                    },
                ],
            )
        )

    mission = Node(
        package="vehicle_loopback_sim",
        executable="loopback_mission_node",
        namespace="comparison",
        name="vehicle_loopback_mission",
        output="screen",
        condition=IfCondition(plan_on_start),
        parameters=[
            mission_parameters,
            {
                "arena_config": ParameterValue(arena_config, value_type=str),
                "mode": ParameterValue(planner_mode, value_type=str),
                "target_ids": ParameterValue(target_ids, value_type=str),
                "return_home": ParameterValue(return_home, value_type=bool),
                "allow_partial": True,
                "planner_service": "/new_map_planner/plan",
                "request_delay_s": ParameterValue(plan_request_delay, value_type=float),
                "status_topic": "/comparison/mission_status",
                "navigator_status_topic": "/comparison/new/navigator_status",
                "navigator_status_topics": (
                    "/comparison/new/navigator_status,"
                    "/comparison/old/navigator_status"
                ),
                "odom_topic": "/comparison/new/odometry/fused",
            },
        ],
    )

    gui_nodes = []
    for controller, label, port in (
        ("new", "New shunt controller", new_gui_port),
        ("old", "Legacy in-place-turn controller", old_gui_port),
    ):
        gui_nodes.append(
            Node(
                package="vehicle_loopback_sim",
                executable="vehicle_loopback_gui_node",
                namespace=f"comparison/{controller}",
                name="vehicle_loopback_gui",
                output="screen",
                condition=IfCondition(gui),
                parameters=[
                    gui_parameters,
                    _gui_topics(controller),
                    {
                        "display_name": label,
                        "port": ParameterValue(port, value_type=int),
                        "arena_config": ParameterValue(arena_config, value_type=str),
                    },
                ],
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_config", default_value=default_arena),
            DeclareLaunchArgument("planner_mode", default_value="layer1,layer2,layer3"),
            DeclareLaunchArgument(
                "target_ids", default_value="1,2,3,4,5,6,7,8,9,10,11,12"
            ),
            DeclareLaunchArgument("return_home", default_value="true"),
            DeclareLaunchArgument("plan_request_delay", default_value="1.0"),
            DeclareLaunchArgument("plan_on_start", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("new_gui_port", default_value="8765"),
            DeclareLaunchArgument("old_gui_port", default_value="8766"),
            planner,
            new_navigator,
            old_navigator,
            *simulators,
            mission,
            *gui_nodes,
        ]
    )
