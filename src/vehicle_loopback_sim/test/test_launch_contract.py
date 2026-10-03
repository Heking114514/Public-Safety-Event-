from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_loopback_launch_defaults_to_formal_waypoint_navigator():
    launch_text = (
        PACKAGE_ROOT / "launch" / "planner_control_loopback.launch.py"
    ).read_text()

    assert '"use_waypoint_navigator",\n                default_value="true"' in launch_text
    assert '"use_debug_controller",\n                default_value="false"' in launch_text
    assert 'default_value="layer1,layer2,layer3"' in launch_text


def test_loopback_package_declares_formal_navigation_dependency():
    package_xml = ET.parse(PACKAGE_ROOT / "package.xml")
    dependencies = {
        element.text
        for element in package_xml.getroot().findall("exec_depend")
    }

    assert "visual_navigation" in dependencies
