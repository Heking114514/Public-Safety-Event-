from dataclasses import replace
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import yaml

from vehicle_config import (
    camera_link_rotation_matrix,
    complete_mount_translation,
    load_vehicle_physical_config,
    parameters_for,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
CENTRAL_CONFIG = (
    REPOSITORY_ROOT / "src" / "vehicle_config" / "config" / "vehicle_physical.yaml"
)
VEHICLE_CONFIG_CONSUMERS = {
    "arena_path_planner",
    "fused_odometry",
    "pure_line_following",
    "rectangle_odometry_test",
    "vision_correction",
    "visual_navigation",
    "wheel_odometry",
}
CENTRAL_PARAMETER_KEYS = {
    "base_offset_x_m",
    "base_offset_y_m",
    "camera_height_m",
    "camera_pitch_rad",
    "camera_roll_rad",
    "camera_x_m",
    "camera_y_m",
    "camera_yaw_rad",
    "left_distance_scale",
    "left_encoder_counts_per_revolution",
    "max_wheel_speed_mps",
    "obstacle_sensor_forward_offset",
    "right_distance_scale",
    "right_encoder_counts_per_revolution",
    "tracking_point_offset_x",
    "tracking_point_offset_y",
    "vehicle_front_offset",
    "vehicle_length_m",
    "vehicle_width_m",
    "wheel_base_offset_x_m",
    "wheel_radius_m",
    "wheel_track_m",
    "yaw_slip_scale",
}


def _mapping_keys(value):
    if isinstance(value, dict):
        for key, child in value.items():
            yield str(key)
            yield from _mapping_keys(child)
    elif isinstance(value, list):
        for child in value:
            yield from _mapping_keys(child)


def _package_dependencies(package: str) -> set[str]:
    package_xml = REPOSITORY_ROOT / "src" / package / "package.xml"
    root = ElementTree.fromstring(package_xml.read_text(encoding="utf-8"))
    dependency_tags = {
        "depend",
        "build_depend",
        "build_export_depend",
        "exec_depend",
        "test_depend",
    }
    return {
        element.text.strip()
        for element in root
        if element.tag in dependency_tags and element.text and element.text.strip()
    }


def test_measured_camera_mount_uses_ros_body_axes():
    physical = load_vehicle_physical_config(CENTRAL_CONFIG)

    assert physical.camera_x_m == 0.055
    assert physical.camera_y_m == -0.015
    assert physical.camera_z_m == 0.121


def test_component_mappings_follow_one_changed_physical_model():
    physical = replace(
        load_vehicle_physical_config(CENTRAL_CONFIG),
        wheel_track_m=0.2222,
        base_link_offset_x_m=0.061,
        base_link_offset_y_m=-0.004,
        tracking_point_x_m=0.037,
        camera_y_m=-0.026,
    )

    assert parameters_for("wheel_odometry", physical)["wheel_track_m"] == 0.2222
    assert parameters_for("pure_line_following", physical)["wheel_track_m"] == 0.2222
    assert parameters_for("pure_line_following", physical)["base_offset_x_m"] == (
        physical.base_link_offset_x_m
    )
    assert parameters_for("pure_line_following", physical)["base_offset_y_m"] == (
        physical.base_link_offset_y_m
    )
    assert parameters_for("pure_line_following", physical)["vehicle_length_m"] == (
        physical.length_m
    )
    assert (
        parameters_for("rectangle_primitive_controller", physical)["wheel_track_m"]
        == 0.2222
    )
    assert (
        parameters_for("waypoint_navigation", physical)["tracking_point_offset_x"]
        == 0.037
    )
    assert parameters_for("waypoint_navigation", physical)["vehicle_length_m"] == (
        physical.length_m
    )
    assert parameters_for("waypoint_navigation", physical)["vehicle_width_m"] == (
        physical.width_m
    )
    assert parameters_for("waypoint_navigation", physical)["wheel_track_m"] == 0.2222
    assert parameters_for("waypoint_navigation", physical)["base_offset_x_m"] == 0.061
    assert parameters_for("waypoint_navigation", physical)["base_offset_y_m"] == -0.004
    assert parameters_for("pure_line_following", physical)["camera_y_m"] == -0.026
    assert parameters_for("vision_correction", physical)["camera_y_m"] == -0.026


def test_camera_rotation_is_a_valid_rotation_matrix():
    rotation = np.asarray(
        camera_link_rotation_matrix(load_vehicle_physical_config(CENTRAL_CONFIG))
    )

    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1.0e-12)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-12)


def test_complete_tf_height_is_not_added_twice_and_legacy_zeros_are_filled():
    configured = (0.055, -0.015, 0.121)

    assert complete_mount_translation(configured, configured) == configured
    assert complete_mount_translation((0.055, 0.0, 0.0), configured) == configured


def test_consuming_packages_declare_vehicle_config_dependency():
    missing = [
        package
        for package in sorted(VEHICLE_CONFIG_CONSUMERS)
        if "vehicle_config" not in _package_dependencies(package)
    ]

    assert missing == []


def test_other_project_yaml_files_do_not_redeclare_central_parameters():
    duplicates = []
    paths = list((REPOSITORY_ROOT / "config").glob("*.yaml"))
    paths.extend((REPOSITORY_ROOT / "src").glob("*/config/*.yaml"))
    for path in sorted(paths):
        if path == CENTRAL_CONFIG:
            continue
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        repeated = sorted(set(_mapping_keys(document)) & CENTRAL_PARAMETER_KEYS)
        if repeated:
            duplicates.append(f"{path.relative_to(REPOSITORY_ROOT)}: {', '.join(repeated)}")

    assert duplicates == []
