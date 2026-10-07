from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def test_bringup_starts_planner_manager_and_ui_without_ui_owning_lifetime():
    text = (PACKAGE / "launch" / "mission_navigation.launch.py").read_text()

    assert 'package="new_map_planner"' in text
    assert 'executable="new_map_planner_node"' in text
    assert "mission_manager.launch.py" in text
    assert 'executable="new_map_planner_ui"' in text
    assert "OnProcessExit" not in text
    assert "vehicle_loopback_sim" not in text
    assert "reporter_stub_node" not in text


def test_manager_does_not_publish_velocity_commands():
    text = (PACKAGE / "navigation_mission_manager" / "node.py").read_text()

    assert "/cmd_vel" not in text
    assert "Twist" not in text


def test_manager_dispatches_only_current_planner_response():
    text = (PACKAGE / "navigation_mission_manager" / "node.py").read_text()

    assert "request.activate = False" in text
    assert "self._route_publisher.publish(plan.navigation_path)" in text
    assert "generation != self._plan_generation" in text
