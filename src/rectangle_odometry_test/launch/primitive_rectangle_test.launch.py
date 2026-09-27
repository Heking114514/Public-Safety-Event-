from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="/odometry/fused"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_nav"),
            DeclareLaunchArgument("route_frame", default_value="map"),
            DeclareLaunchArgument("child_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "fusion_status_topic", default_value="/odometry/fusion_status"
            ),
            DeclareLaunchArgument("require_fusion_status", default_value="true"),
            DeclareLaunchArgument("fusion_status_timeout_s", default_value="0.8"),
            DeclareLaunchArgument(
                "actuator_health_topic",
                default_value="/cup_car_serial/actuator_healthy",
            ),
            DeclareLaunchArgument("require_actuator_health", default_value="true"),
            DeclareLaunchArgument("actuator_health_timeout_s", default_value="0.8"),
            DeclareLaunchArgument(
                "obstacle_topic", default_value="/obstacle/front_blocked"
            ),
            DeclareLaunchArgument("obstacle_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "motion_hold_state_topic",
                default_value="/waypoint_navigation/motion_hold_state",
            ),
            DeclareLaunchArgument("motion_hold_timeout_s", default_value="0.8"),
            DeclareLaunchArgument("side_length_m", default_value="1.8"),
            DeclareLaunchArgument("control_period_s", default_value="0.05"),
            DeclareLaunchArgument("odom_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("translation_step_m", default_value="0.03"),
            DeclareLaunchArgument("primitive_duration_s", default_value="0.50"),
            DeclareLaunchArgument("position_resolution_m", default_value="0.01"),
            DeclareLaunchArgument("yaw_resolution_rad", default_value="0.0872665"),
            DeclareLaunchArgument("search_margin_m", default_value="0.4"),
            DeclareLaunchArgument("max_expansions", default_value="20000"),
            DeclareLaunchArgument(
                "planner_goal_position_tolerance_m", default_value="0.04"
            ),
            DeclareLaunchArgument(
                "planner_goal_yaw_tolerance_rad", default_value="0.0698132"
            ),
            DeclareLaunchArgument("position_tolerance_m", default_value="0.05"),
            DeclareLaunchArgument("yaw_tolerance_rad", default_value="0.0872665"),
            DeclareLaunchArgument("replan_position_error_m", default_value="0.10"),
            DeclareLaunchArgument("replan_yaw_error_rad", default_value="0.20944"),
            DeclareLaunchArgument("replan_cooldown_s", default_value="0.30"),
            DeclareLaunchArgument("failed_plan_retry_s", default_value="0.50"),
            DeclareLaunchArgument("goal_line_tolerance_m", default_value="0.05"),
            DeclareLaunchArgument("goal_lateral_tolerance_m", default_value="0.10"),
            DeclareLaunchArgument(
                "turn_correction_start_rad", default_value="0.10472"
            ),
            DeclareLaunchArgument(
                "turn_correction_min_rate_radps", default_value="0.174533"
            ),
            DeclareLaunchArgument(
                "turn_correction_max_rate_radps", default_value="0.436332"
            ),
            DeclareLaunchArgument(
                "turn_correction_gain", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "straight_heading_deadband_rad", default_value="0.02618"
            ),
            DeclareLaunchArgument(
                "straight_heading_gain", default_value="1.5"
            ),
            DeclareLaunchArgument(
                "straight_heading_max_rate_radps", default_value="0.139626"
            ),
            Node(
                package="rectangle_odometry_test",
                executable="primitive_controller_node",
                name="rectangle_primitive_controller",
                output="screen",
                parameters=[
                    {
                        "enable": ParameterValue(
                            LaunchConfiguration("enable"), value_type=bool
                        ),
                        "odom_topic": ParameterValue(
                            LaunchConfiguration("odom_topic"), value_type=str
                        ),
                        "cmd_vel_topic": ParameterValue(
                            LaunchConfiguration("cmd_vel_topic"), value_type=str
                        ),
                        "route_frame": ParameterValue(
                            LaunchConfiguration("route_frame"), value_type=str
                        ),
                        "child_frame": ParameterValue(
                            LaunchConfiguration("child_frame"), value_type=str
                        ),
                        "fusion_status_topic": ParameterValue(
                            LaunchConfiguration("fusion_status_topic"), value_type=str
                        ),
                        "require_fusion_status": ParameterValue(
                            LaunchConfiguration("require_fusion_status"),
                            value_type=bool,
                        ),
                        "fusion_status_timeout_s": ParameterValue(
                            LaunchConfiguration("fusion_status_timeout_s"),
                            value_type=float,
                        ),
                        "actuator_health_topic": ParameterValue(
                            LaunchConfiguration("actuator_health_topic"),
                            value_type=str,
                        ),
                        "require_actuator_health": ParameterValue(
                            LaunchConfiguration("require_actuator_health"),
                            value_type=bool,
                        ),
                        "actuator_health_timeout_s": ParameterValue(
                            LaunchConfiguration("actuator_health_timeout_s"),
                            value_type=float,
                        ),
                        "obstacle_topic": ParameterValue(
                            LaunchConfiguration("obstacle_topic"), value_type=str
                        ),
                        "obstacle_timeout_s": ParameterValue(
                            LaunchConfiguration("obstacle_timeout_s"),
                            value_type=float,
                        ),
                        "motion_hold_state_topic": ParameterValue(
                            LaunchConfiguration("motion_hold_state_topic"),
                            value_type=str,
                        ),
                        "motion_hold_timeout_s": ParameterValue(
                            LaunchConfiguration("motion_hold_timeout_s"),
                            value_type=float,
                        ),
                        "side_length_m": ParameterValue(
                            LaunchConfiguration("side_length_m"), value_type=float
                        ),
                        "control_period_s": ParameterValue(
                            LaunchConfiguration("control_period_s"), value_type=float
                        ),
                        "odom_timeout_s": ParameterValue(
                            LaunchConfiguration("odom_timeout_s"), value_type=float
                        ),
                        "translation_step_m": ParameterValue(
                            LaunchConfiguration("translation_step_m"),
                            value_type=float,
                        ),
                        "primitive_duration_s": ParameterValue(
                            LaunchConfiguration("primitive_duration_s"),
                            value_type=float,
                        ),
                        "position_resolution_m": ParameterValue(
                            LaunchConfiguration("position_resolution_m"),
                            value_type=float,
                        ),
                        "yaw_resolution_rad": ParameterValue(
                            LaunchConfiguration("yaw_resolution_rad"),
                            value_type=float,
                        ),
                        "search_margin_m": ParameterValue(
                            LaunchConfiguration("search_margin_m"),
                            value_type=float,
                        ),
                        "max_expansions": ParameterValue(
                            LaunchConfiguration("max_expansions"), value_type=int
                        ),
                        "planner_goal_position_tolerance_m": ParameterValue(
                            LaunchConfiguration(
                                "planner_goal_position_tolerance_m"
                            ),
                            value_type=float,
                        ),
                        "planner_goal_yaw_tolerance_rad": ParameterValue(
                            LaunchConfiguration(
                                "planner_goal_yaw_tolerance_rad"
                            ),
                            value_type=float,
                        ),
                        "position_tolerance_m": ParameterValue(
                            LaunchConfiguration("position_tolerance_m"),
                            value_type=float,
                        ),
                        "yaw_tolerance_rad": ParameterValue(
                            LaunchConfiguration("yaw_tolerance_rad"), value_type=float
                        ),
                        "replan_position_error_m": ParameterValue(
                            LaunchConfiguration("replan_position_error_m"),
                            value_type=float,
                        ),
                        "replan_yaw_error_rad": ParameterValue(
                            LaunchConfiguration("replan_yaw_error_rad"),
                            value_type=float,
                        ),
                        "replan_cooldown_s": ParameterValue(
                            LaunchConfiguration("replan_cooldown_s"),
                            value_type=float,
                        ),
                        "failed_plan_retry_s": ParameterValue(
                            LaunchConfiguration("failed_plan_retry_s"),
                            value_type=float,
                        ),
                        "goal_line_tolerance_m": ParameterValue(
                            LaunchConfiguration("goal_line_tolerance_m"),
                            value_type=float,
                        ),
                        "goal_lateral_tolerance_m": ParameterValue(
                            LaunchConfiguration("goal_lateral_tolerance_m"),
                            value_type=float,
                        ),
                        "turn_correction_start_rad": ParameterValue(
                            LaunchConfiguration("turn_correction_start_rad"),
                            value_type=float,
                        ),
                        "turn_correction_min_rate_radps": ParameterValue(
                            LaunchConfiguration(
                                "turn_correction_min_rate_radps"
                            ),
                            value_type=float,
                        ),
                        "turn_correction_max_rate_radps": ParameterValue(
                            LaunchConfiguration(
                                "turn_correction_max_rate_radps"
                            ),
                            value_type=float,
                        ),
                        "turn_correction_gain": ParameterValue(
                            LaunchConfiguration("turn_correction_gain"),
                            value_type=float,
                        ),
                        "straight_heading_deadband_rad": ParameterValue(
                            LaunchConfiguration(
                                "straight_heading_deadband_rad"
                            ),
                            value_type=float,
                        ),
                        "straight_heading_gain": ParameterValue(
                            LaunchConfiguration("straight_heading_gain"),
                            value_type=float,
                        ),
                        "straight_heading_max_rate_radps": ParameterValue(
                            LaunchConfiguration(
                                "straight_heading_max_rate_radps"
                            ),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
