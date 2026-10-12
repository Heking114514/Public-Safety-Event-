import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "contrast_comparison.launch.py"
LAUNCH_TEXT = LAUNCH_PATH.read_text(encoding="utf-8")
LAUNCH_TREE = ast.parse(LAUNCH_TEXT, filename=str(LAUNCH_PATH))


def _topic_helper(name: str):
    """只执行纯话题映射函数，静态测试不依赖 ROS launch 环境。"""
    function = next(
        node
        for node in LAUNCH_TREE.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {}
    exec(compile(module, str(LAUNCH_PATH), "exec"), namespace)
    return namespace[name]


def _string_literals() -> set[str]:
    return {
        node.value
        for node in ast.walk(LAUNCH_TREE)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }


def test_launch_uses_one_planner_and_requested_controllers():
    assert LAUNCH_TEXT.count('package="new_map_planner"') == 1
    assert 'executable="new_map_planner_node"' in LAUNCH_TEXT

    assert 'package="visual_navigation"' in LAUNCH_TEXT
    assert 'executable="waypoint_navigator"' in LAUNCH_TEXT
    assert 'namespace="comparison/production"' in LAUNCH_TEXT

    assert 'package="contrast_controller"' in LAUNCH_TEXT
    assert 'executable="contrast_controller"' in LAUNCH_TEXT
    assert 'namespace="comparison/contrast"' in LAUNCH_TEXT


def test_controller_topics_share_only_route_input():
    controller_topics = _topic_helper("_controller_topics")
    production = controller_topics("production")
    contrast = controller_topics("contrast")

    assert production["route_input_topic"] == "/waypoint_navigation/route_input"
    assert contrast["route_input_topic"] == "/waypoint_navigation/route_input"

    isolated_keys = set(production) - {"route_input_topic"}
    assert isolated_keys == set(contrast) - {"route_input_topic"}
    assert {
        "cmd_vel_topic",
        "odom_topic",
        "imu_topic",
        "control_telemetry_topic",
        "status_topic",
        "current_waypoint_topic",
        "route_ack_topic",
    } <= isolated_keys
    for key in isolated_keys:
        assert production[key].startswith("/comparison/production/")
        assert contrast[key].startswith("/comparison/contrast/")
        assert production[key] != contrast[key]


def test_simulator_and_gui_topics_follow_each_controller():
    controller_topics = _topic_helper("_controller_topics")
    sim_topics = _topic_helper("_sim_topics")
    gui_topics = _topic_helper("_gui_topics")

    for controller in ("production", "contrast"):
        navigator = controller_topics(controller)
        simulator = sim_topics(controller)
        gui = gui_topics(controller)
        root = f"/comparison/{controller}/"

        for key in (
            "cmd_vel_topic",
            "odom_topic",
            "imu_topic",
            "fusion_status_topic",
            "actuator_health_topic",
            "control_telemetry_topic",
            "front_obstacle_topic",
            "front_obstacle_range_topic",
        ):
            assert simulator[key] == navigator[key]

        assert gui["cmd_vel_topic"] == navigator["cmd_vel_topic"]
        assert gui["odom_topic"] == navigator["odom_topic"]
        assert gui["control_telemetry_topic"] == navigator[
            "control_telemetry_topic"
        ]
        assert gui["navigator_status_topic"] == navigator["status_topic"]
        assert gui["current_waypoint_topic"] == navigator[
            "current_waypoint_topic"
        ]
        assert gui["waypoint_path_topic"] == navigator["path_topic"]
        assert gui["loopback_status_topic"] == simulator["status_topic"]
        assert gui["manual_obstacle_topic"] == simulator[
            "manual_obstacle_topic"
        ]
        assert gui["clear_manual_obstacles_topic"] == simulator[
            "clear_manual_obstacles_topic"
        ]
        assert all(topic.startswith(root) for topic in simulator.values())


def test_simulators_share_pose_and_physical_contract():
    assert 'for controller in CONTROLLERS' in LAUNCH_TEXT
    assert 'executable="vehicle_loopback_sim_node"' in LAUNCH_TEXT
    assert 'shared_sim_parameters' in LAUNCH_TEXT
    assert '"publish_tf": False' in LAUNCH_TEXT
    assert '"enforce_static_collision": True' in LAUNCH_TEXT

    for name in ("initial_x", "initial_y", "initial_yaw"):
        assert f'{name} = LaunchConfiguration("{name}")' in LAUNCH_TEXT
    for parameter in ("initial_x_m", "initial_y_m", "initial_yaw_rad"):
        assert f'"{parameter}": ParameterValue(' in LAUNCH_TEXT

    assert 'parameters_for("pure_line_following")' in LAUNCH_TEXT
    for parameter in (
        "wheel_track_m",
        "base_offset_x_m",
        "base_offset_y_m",
        "obstacle_sensor_x_m",
        "obstacle_sensor_y_m",
    ):
        assert f'"{parameter}": simulator_physical[' in LAUNCH_TEXT


def test_mission_waits_for_both_goal_reached_statuses():
    literals = _string_literals()

    assert 'executable="comparison_mission.py"' in LAUNCH_TEXT
    assert '"navigator_status_topics"' in LAUNCH_TEXT
    assert (
        "/comparison/production/navigator_status,"
        "/comparison/contrast/navigator_status"
    ) in literals
    assert '"odom_topic": "/comparison/production/odometry/fused"' in LAUNCH_TEXT


def test_parameters_come_from_package_yaml_and_vehicle_config():
    assert 'production_share / "config" / "waypoint_navigation.yaml"' in LAUNCH_TEXT
    assert 'contrast_share / "config" / "contrast_controller.yaml"' in LAUNCH_TEXT
    assert 'simulator_share / "config" / "vehicle_loopback.yaml"' in LAUNCH_TEXT
    assert 'production_config, "waypoint_navigator"' in LAUNCH_TEXT
    assert 'contrast_config, "contrast_controller"' in LAUNCH_TEXT
    assert (
        "production_parameters,\n"
        "            contrast_parameters,\n"
        "            navigation_physical,\n"
        '            _controller_topics("contrast"),'
    ) in LAUNCH_TEXT
    assert 'parameters_for("waypoint_navigation")' in LAUNCH_TEXT
    assert 'parameters_for("pure_line_following")' in LAUNCH_TEXT
    assert 'vehicle_share / "config" / "vehicle_physical.yaml"' in LAUNCH_TEXT


def test_gui_defaults_use_separate_ports_and_short_labels():
    assert (
        'DeclareLaunchArgument("production_gui_port", default_value="8765")'
        in LAUNCH_TEXT
    )
    assert (
        'DeclareLaunchArgument("contrast_gui_port", default_value="8766")'
        in LAUNCH_TEXT
    )
    assert '("production", "Production", production_gui_port)' in LAUNCH_TEXT
    assert '("contrast", "Contrast", contrast_gui_port)' in LAUNCH_TEXT
