import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "final_navigation_comparison.launch.py"
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


def test_launch_uses_final_navigation_controller():
    assert LAUNCH_TEXT.count('package="new_map_planner"') == 1
    assert 'executable="new_map_planner_node"' in LAUNCH_TEXT
    assert 'package="final_navigation"' in LAUNCH_TEXT
    assert 'executable="final_navigation"' in LAUNCH_TEXT
    assert 'namespace="comparison/final_navigation"' in LAUNCH_TEXT
    assert 'package="vehicle_loopback_sim"' in LAUNCH_TEXT
    assert 'executable="final_navigation_mission_node"' in LAUNCH_TEXT


def test_final_navigation_topics_are_namespaced():
    topics = _topic_helper("_final_navigation_topics")()
    assert topics["route_input_topic"] == "/waypoint_navigation/route_input"
    for key, value in topics.items():
        if key != "route_input_topic":
            assert value.startswith("/comparison/final_navigation/")


def test_simulator_and_gui_topics_follow_controller():
    controller = _topic_helper("_final_navigation_topics")()
    simulator = _topic_helper("_sim_topics")()
    gui = _topic_helper("_gui_topics")()
    assert simulator["cmd_vel_topic"] == controller["cmd_vel_topic"]
    assert simulator["odom_topic"] == controller["odom_topic"]
    assert gui["navigator_status_topic"] == controller["status_topic"]
    assert gui["waypoint_path_topic"] == controller["path_topic"]


def test_simulator_uses_shared_vehicle_contract():
    assert 'namespace="comparison/final_navigation"' in LAUNCH_TEXT
    assert 'CONTROLLERS' not in LAUNCH_TEXT
    assert '"controllers"' not in LAUNCH_TEXT
    assert 'executable="vehicle_loopback_sim_node"' in LAUNCH_TEXT
    assert 'shared_sim_parameters' in LAUNCH_TEXT
    assert 'parameters_for("pure_line_following")' in LAUNCH_TEXT
    assert '"wheel_track_m": simulator_physical[' in LAUNCH_TEXT


def test_mission_uses_final_navigation_topics():
    assert 'executable="final_navigation_mission_node"' in LAUNCH_TEXT
    assert '"/comparison/final_navigation/navigator_status"' in LAUNCH_TEXT
    assert '"/comparison/final_navigation/odometry/fused"' in LAUNCH_TEXT


def test_parameters_come_from_final_navigation_yaml():
    assert 'final_config = final_share / "config" / "final_navigation.yaml"' in LAUNCH_TEXT
    assert '_ros_parameters(final_config, "final_navigation")' in LAUNCH_TEXT
    assert 'parameters_for("waypoint_navigation")' in LAUNCH_TEXT
    assert 'parameters_for("pure_line_following")' in LAUNCH_TEXT
