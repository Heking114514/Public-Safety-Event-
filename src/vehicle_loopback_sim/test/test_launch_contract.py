from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_loopback_launch_defaults_to_formal_waypoint_navigator():
    launch_text = (
        PACKAGE_ROOT / "launch" / "planner_control_loopback.launch.py"
    ).read_text()

    assert (
        '"use_waypoint_navigator",\n                default_value="true"' in launch_text
    )
    assert (
        '"use_debug_controller",\n                default_value="false"' in launch_text
    )
    assert 'default_value="layer1,layer2,layer3"' in launch_text
    assert 'default_value="1,2,3,4,5,6,7,8,9,10,11,12"' in launch_text
    assert 'DeclareLaunchArgument("return_home", default_value="true")' in launch_text
    assert 'DeclareLaunchArgument("allow_partial", default_value="true")' in launch_text
    assert 'default_value="false"' in launch_text
    assert 'playback_speed = LaunchConfiguration("playback_speed")' in launch_text
    assert '"control_rate_multiplier": playback_speed' in launch_text
    assert '"playback_speed": ParameterValue(playback_speed' in launch_text
    assert 'package="new_map_planner"' in launch_text
    assert 'executable="new_map_planner_node"' in launch_text
    assert 'package="navigation_mission_manager"' in launch_text
    assert 'executable="mission_manager_node"' in launch_text
    assert 'name="navigation_mission_manager"' in launch_text
    assert '"task_confirmation_mode": "route"' in launch_text
    assert 'executable="loopback_mission_node"' not in launch_text
    assert 'name="vehicle_loopback_mission"' not in launch_text


def test_legacy_playback_remains_separate_from_shared_simulation_clock():
    for launch_name in ("vehicle_loopback.launch.py", "planner_control_loopback.launch.py"):
        launch_text = (PACKAGE_ROOT / "launch" / launch_name).read_text()
        assert 'LaunchConfiguration("simulation_mode")' in launch_text
        assert 'default_value="closed_loop"' in launch_text
        assert '"simulation_mode": ParameterValue(' in launch_text

    simulator_text = (PACKAGE_ROOT / "src" / "simulator.cpp").read_text()
    assert 'declare_parameter("simulation_mode", "closed_loop")' in simulator_text
    assert "closed_loop requires playback_speed=1.0" in simulator_text
    assert "NON-ACCEPTANCE" in simulator_text

    config_text = (PACKAGE_ROOT / "config" / "vehicle_loopback.yaml").read_text()
    assert "simulation_mode: closed_loop" in config_text


def test_loopback_package_declares_formal_navigation_dependency():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml")
    dependencies = {
        element.text for element in package_xml.getroot().findall("exec_depend")
    }

    assert "visual_navigation" in dependencies
    assert "new_map_planner" in dependencies
    assert "navigation_mission_manager" in dependencies
    assert "arena_path_planner" not in dependencies


def test_mixed_package_builds_runtime_simulator_in_cpp():
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text()

    assert "add_executable(vehicle_loopback_sim_node" in cmake
    assert "ament_python_install_package(vehicle_loopback_sim)" in cmake


def test_loopback_sensor_does_not_treat_static_map_as_dynamic_obstacle():
    node_text = (PACKAGE_ROOT / "src" / "sim_obstacle.cpp").read_text()

    assert "layout_obstacles" not in node_text
    assert "manual_obstacles_" in node_text


def test_gui_labels_reference_route_separately_from_actual_trail():
    page = (PACKAGE_ROOT / "vehicle_loopback_sim" / "web" / "index.html").read_text()
    gui = (PACKAGE_ROOT / "vehicle_loopback_sim" / "gui.py").read_text()

    assert "导航器当前路线" in page
    assert "实际车迹" in page
    assert "function pathSummary(path)" in page
    assert '"stamp_ns"' in gui


def test_comparison_launch_runs_isolated_new_and_legacy_controllers():
    launch_text = (
        PACKAGE_ROOT / "launch" / "controller_comparison.launch.py"
    ).read_text()

    assert 'executable="waypoint_navigator"' in launch_text
    assert 'executable="waypoint_navigator_legacy"' in launch_text
    assert 'namespace="comparison/new"' in launch_text
    assert 'namespace="comparison/old"' in launch_text
    assert 'DeclareLaunchArgument("new_gui_port", default_value="8765")' in launch_text
    assert 'DeclareLaunchArgument("old_gui_port", default_value="8766")' in launch_text
    assert '"navigator_status_topics"' in launch_text
    assert 'package="new_map_planner"' in launch_text
