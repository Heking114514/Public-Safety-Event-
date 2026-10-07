from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_config import parameters_for


def generate_launch_description():
    arena_config = str(
        Path(get_package_share_directory("new_map_planner")) / "config" / "arena.yaml"
    )
    waypoint_launch = str(
        Path(get_package_share_directory("visual_navigation"))
        / "launch"
        / "waypoint_navigation.launch.py"
    )
    loopback_parameters = str(
        Path(get_package_share_directory("vehicle_loopback_sim"))
        / "config"
        / "vehicle_loopback.yaml"
    )
    mission_config = str(
        Path(get_package_share_directory("navigation_mission_manager"))
        / "config"
        / "mission_manager.yaml"
    )

    planner_config = LaunchConfiguration("planner_config")
    loopback_parameters_file = LaunchConfiguration("loopback_parameters_file")
    route_file = LaunchConfiguration("route_file")
    autostart = LaunchConfiguration("autostart")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    gui = LaunchConfiguration("gui")
    gui_host = LaunchConfiguration("gui_host")
    gui_port = LaunchConfiguration("gui_port")
    plan_on_start = LaunchConfiguration("plan_on_start")
    target_ids = LaunchConfiguration("target_ids")
    use_debug_controller = LaunchConfiguration("use_debug_controller")
    use_waypoint_navigator = LaunchConfiguration("use_waypoint_navigator")
    simulate_obstacle_recognition = LaunchConfiguration(
        "simulate_obstacle_recognition"
    )
    physical = parameters_for("pure_line_following")

    planner = Node(
        package="new_map_planner",
        executable="new_map_planner_node",
        name="new_map_planner",
        output="screen",
        parameters=[{"map_file": planner_config}],
    )

    navigator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(waypoint_launch),
        condition=IfCondition(use_waypoint_navigator),
        launch_arguments={
            "route_file": route_file,
            "route_frame": "map",
            "odom_topic": "/odometry/fused",
            "fusion_status_topic": "/odometry/fusion_status",
            "cmd_vel_topic": "/cmd_vel_nav",
            "autostart": autostart,
        }.items(),
    )

    debug_controller = Node(
        package="vehicle_loopback_sim",
        executable="loopback_route_controller",
        name="loopback_route_controller",
        output="screen",
        condition=IfCondition(use_debug_controller),
        parameters=[
            loopback_parameters_file,
            {
                "route_input_topic": "/waypoint_navigation/route_input",
                "odom_topic": "/odometry/fused",
                "cmd_vel_topic": "/cmd_vel_nav",
                "path_topic": "/waypoint_path",
                "status_topic": "/waypoint_navigation/status",
                "current_waypoint_topic": "/waypoint_navigation/current_waypoint",
            },
        ],
    )

    loopback = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_sim_node",
        name="vehicle_loopback_sim",
        output="screen",
        parameters=[
            loopback_parameters_file,
            {
                "cmd_vel_topic": "/cmd_vel_nav",
                "odom_topic": "/odometry/fused",
                "arena_config": ParameterValue(planner_config, value_type=str),
                "initial_x_m": ParameterValue(initial_x, value_type=float),
                "initial_y_m": ParameterValue(initial_y, value_type=float),
                "initial_yaw_rad": ParameterValue(initial_yaw, value_type=float),
                "wheel_track_m": physical["wheel_track_m"],
                "base_offset_x_m": physical["base_offset_x_m"],
                "base_offset_y_m": physical["base_offset_y_m"],
                "publish_obstacle_detection": ParameterValue(
                    simulate_obstacle_recognition, value_type=bool
                ),
            },
        ],
    )

    gui_node = Node(
        package="vehicle_loopback_sim",
        executable="vehicle_loopback_gui_node",
        name="vehicle_loopback_gui",
        output="screen",
        condition=IfCondition(gui),
        parameters=[
            loopback_parameters_file,
            {
                "host": ParameterValue(gui_host, value_type=str),
                "port": ParameterValue(gui_port, value_type=int),
                "arena_config": ParameterValue(planner_config, value_type=str),
                "cmd_vel_topic": "/cmd_vel_nav",
                "odom_topic": "/odometry/fused",
                "navigation_path_topic": "/new_map_planner/navigation_path",
                "arena_path_topic": "/new_map_planner/arena_path",
                "map_topic": "/new_map_planner/map",
            },
        ],
    )

    mission = Node(
        package="navigation_mission_manager",
        executable="mission_manager_node",
        name="navigation_mission_manager",
        output="screen",
        condition=IfCondition(plan_on_start),
        parameters=[
            mission_config,
            {
                "map_file": ParameterValue(planner_config, value_type=str),
                "auto_start": True,
                "target_ids": ParameterValue(target_ids, value_type=str),
                "planner_service": "/new_map_planner/plan",
                "odom_topic": "/odometry/fused",
                "navigator_status_topic": "/waypoint_navigation/status",
                "route_ack_topic": "/waypoint_navigation/route_ack",
                "motion_hold_service": "/waypoint_navigator/set_motion_hold",
                "motion_hold_state_topic": (
                    "/waypoint_navigation/motion_hold_state"
                ),
                "front_obstacle_topic": "/obstacle/front_blocked",
                "front_range_topic": "/obstacle/front_range",
                "status_text_topic": (
                    "/vehicle_loopback_sim/mission_status"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_config", default_value=arena_config),
            DeclareLaunchArgument(
                "loopback_parameters_file",
                default_value=loopback_parameters,
                description="Vehicle loopback parameter file",
            ),
            DeclareLaunchArgument(
                "route_file",
                default_value="",
                description="Optional CSV route; empty waits for planner route input",
            ),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the browser visualization node",
            ),
            DeclareLaunchArgument("gui_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("gui_port", default_value="8765"),
            DeclareLaunchArgument(
                "plan_on_start",
                default_value="true",
                description="Start the formal navigation mission manager",
            ),
            DeclareLaunchArgument(
                "planner_service",
                default_value="/new_map_planner/plan",
                description=(
                    "Legacy loopback mission argument kept for script "
                    "compatibility; ignored by the default mission manager"
                ),
            ),
            DeclareLaunchArgument(
                "planner_mode",
                default_value="layer1,layer2,layer3",
                description=(
                    "Legacy loopback mission argument kept for script "
                    "compatibility; ignored by the default mission manager"
                ),
            ),
            DeclareLaunchArgument(
                "target_ids",
                default_value="1,2,3,4,5,6,7,8,9,10,11,12",
                description="Comma-separated task ids; empty uses planner defaults",
            ),
            DeclareLaunchArgument(
                "required_lane_ids",
                default_value="",
                description=(
                    "Legacy loopback mission argument kept for script "
                    "compatibility; ignored by the default mission manager"
                ),
            ),
            DeclareLaunchArgument("return_home", default_value="true"),
            DeclareLaunchArgument("allow_partial", default_value="true"),
            DeclareLaunchArgument("plan_request_delay", default_value="1.0"),
            DeclareLaunchArgument(
                "use_debug_controller",
                default_value="false",
                description="Use the debug-only loopback route controller",
            ),
            DeclareLaunchArgument(
                "use_waypoint_navigator",
                default_value="true",
                description="Use the formal visual_navigation waypoint navigator",
            ),
            DeclareLaunchArgument(
                "simulate_obstacle_recognition",
                default_value="true",
                description="Publish simulated front obstacle reports from the map",
            ),
            loopback,
            planner,
            debug_controller,
            navigator,
            gui_node,
            mission,
        ]
    )
