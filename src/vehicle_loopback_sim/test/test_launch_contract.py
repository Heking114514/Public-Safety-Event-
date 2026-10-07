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
    assert 'package="new_map_planner"' in launch_text
    assert 'executable="new_map_planner_node"' in launch_text
    assert 'package="navigation_mission_manager"' in launch_text
    assert 'executable="mission_manager_node"' in launch_text
    assert 'name="navigation_mission_manager"' in launch_text
    assert 'executable="loopback_mission_node"' not in launch_text
    assert 'name="vehicle_loopback_mission"' not in launch_text


def test_loopback_package_declares_formal_navigation_dependency():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml")
    dependencies = {
        element.text for element in package_xml.getroot().findall("exec_depend")
    }

    assert "visual_navigation" in dependencies
    assert "new_map_planner" in dependencies
    assert "navigation_mission_manager" in dependencies
    assert "arena_path_planner" not in dependencies


def test_python_package_registers_pytest_with_colcon():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()

    assert 'tests_require=["pytest"]' in setup_text


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
