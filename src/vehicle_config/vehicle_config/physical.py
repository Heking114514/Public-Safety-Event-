from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # Allows repository-local tools to use the source YAML.
    get_package_share_directory = None


@dataclass(frozen=True)
class VehiclePhysicalConfig:
    length_m: float
    width_m: float
    wheel_radius_m: float
    wheel_track_m: float
    left_encoder_counts_per_revolution: float
    right_encoder_counts_per_revolution: float
    left_distance_scale: float
    right_distance_scale: float
    yaw_slip_scale: float
    max_wheel_speed_mps: float
    base_link_offset_x_m: float
    base_link_offset_y_m: float
    tracking_point_x_m: float
    tracking_point_y_m: float
    camera_parent_frame: str
    camera_child_frame: str
    camera_x_m: float
    camera_y_m: float
    camera_z_m: float
    camera_roll_rad: float
    camera_pitch_rad: float
    camera_yaw_rad: float

    @property
    def vehicle_front_offset_m(self) -> float:
        return 0.5 * self.length_m

    @property
    def encoder_counts_per_revolution(self) -> float:
        """Backward-compatible mean of the independently calibrated wheels."""
        return 0.5 * (
            self.left_encoder_counts_per_revolution +
            self.right_encoder_counts_per_revolution
        )


def rpy_rotation_matrix(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[tuple[float, float, float], ...]:
    """Return the ROS fixed-axis roll/pitch/yaw rotation matrix."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def camera_link_rotation_matrix(
    config: VehiclePhysicalConfig | None = None,
) -> tuple[tuple[float, float, float], ...]:
    physical = config or load_vehicle_physical_config()
    return rpy_rotation_matrix(
        physical.camera_roll_rad,
        physical.camera_pitch_rad,
        physical.camera_yaw_rad,
    )


def complete_mount_translation(
    recorded: Sequence[float],
    configured: Sequence[float],
    *,
    missing_tolerance: float = 1.0e-9,
) -> tuple[float, float, float]:
    """Fill axes omitted by legacy zero-valued mount transforms."""
    if len(recorded) != 3 or len(configured) != 3:
        raise ValueError("mount translations must contain exactly three values")
    result = []
    for recorded_value, configured_value in zip(recorded, configured):
        measured = float(recorded_value)
        fallback = float(configured_value)
        if not math.isfinite(measured) or not math.isfinite(fallback):
            raise ValueError("mount translations must be finite")
        if abs(measured) <= missing_tolerance and abs(fallback) > missing_tolerance:
            measured = fallback
        result.append(measured)
    return result[0], result[1], result[2]


def _default_config_path() -> Path:
    if get_package_share_directory is not None:
        try:
            return (
                Path(get_package_share_directory("vehicle_config"))
                / "config"
                / "vehicle_physical.yaml"
            )
        except Exception:
            pass
    return Path(__file__).resolve().parents[1] / "config" / "vehicle_physical.yaml"


def _mapping(parent: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"vehicle physical config requires mapping '{key}'")
    return value


def _finite(mapping: Mapping[str, Any], key: str, *, positive: bool = False) -> float:
    try:
        value = float(mapping[key])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"vehicle physical config requires numeric '{key}'") from error
    if not math.isfinite(value) or (positive and value <= 0.0):
        qualifier = "positive " if positive else "finite "
        raise ValueError(f"vehicle physical config requires {qualifier}'{key}'")
    return value


def _frame(mapping: Mapping[str, Any], key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"vehicle physical config requires non-empty '{key}'")
    return value.strip()


def _nonzero(mapping: Mapping[str, Any], key: str) -> float:
    value = _finite(mapping, key)
    if value == 0.0:
        raise ValueError(f"vehicle physical config requires non-zero '{key}'")
    return value


def load_vehicle_physical_config(
    path: str | Path | None = None,
) -> VehiclePhysicalConfig:
    config_path = Path(path) if path is not None else _default_config_path()
    with config_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)
    if not isinstance(root, Mapping):
        raise ValueError("vehicle physical config root must be a mapping")
    if root.get("schema_version") != 1:
        raise ValueError("vehicle physical config schema_version must be 1")

    chassis = _mapping(root, "chassis")
    drive = _mapping(root, "differential_drive")
    tracking_point = _mapping(root, "tracking_point")
    camera = _mapping(root, "camera")
    return VehiclePhysicalConfig(
        length_m=_finite(chassis, "length_m", positive=True),
        width_m=_finite(chassis, "width_m", positive=True),
        wheel_radius_m=_finite(drive, "wheel_radius_m", positive=True),
        wheel_track_m=_finite(drive, "wheel_track_m", positive=True),
        left_encoder_counts_per_revolution=_finite(
            drive,
            "left_encoder_counts_per_revolution",
            positive=True,
        ),
        right_encoder_counts_per_revolution=_finite(
            drive,
            "right_encoder_counts_per_revolution",
            positive=True,
        ),
        left_distance_scale=_nonzero(drive, "left_distance_scale"),
        right_distance_scale=_nonzero(drive, "right_distance_scale"),
        yaw_slip_scale=_finite(drive, "yaw_slip_scale", positive=True),
        max_wheel_speed_mps=_finite(
            drive, "max_wheel_speed_mps", positive=True
        ),
        base_link_offset_x_m=_finite(drive, "base_link_offset_x_m"),
        base_link_offset_y_m=_finite(drive, "base_link_offset_y_m"),
        tracking_point_x_m=_finite(tracking_point, "x_m"),
        tracking_point_y_m=_finite(tracking_point, "y_m"),
        camera_parent_frame=_frame(camera, "parent_frame"),
        camera_child_frame=_frame(camera, "child_frame"),
        camera_x_m=_finite(camera, "x_m"),
        camera_y_m=_finite(camera, "y_m"),
        camera_z_m=_finite(camera, "z_m", positive=True),
        camera_roll_rad=_finite(camera, "roll_rad"),
        camera_pitch_rad=_finite(camera, "pitch_rad"),
        camera_yaw_rad=_finite(camera, "yaw_rad"),
    )


def parameters_for(
    component: str,
    config: VehiclePhysicalConfig | None = None,
) -> dict[str, float]:
    physical = config or load_vehicle_physical_config()
    mappings = {
        "pure_line_following": {
            "vehicle_length_m": physical.length_m,
            "camera_x_m": physical.camera_x_m,
            "camera_y_m": physical.camera_y_m,
            "camera_height_m": physical.camera_z_m,
            "camera_roll_rad": physical.camera_roll_rad,
            "camera_pitch_rad": physical.camera_pitch_rad,
            "camera_yaw_rad": physical.camera_yaw_rad,
            "vehicle_width_m": physical.width_m,
            "wheel_track_m": physical.wheel_track_m,
            "base_offset_x_m": physical.base_link_offset_x_m,
            "base_offset_y_m": physical.base_link_offset_y_m,
        },
        "vision_correction": {
            "camera_x_m": physical.camera_x_m,
            "camera_y_m": physical.camera_y_m,
            "camera_height_m": physical.camera_z_m,
            "camera_roll_rad": physical.camera_roll_rad,
            "camera_pitch_rad": physical.camera_pitch_rad,
            "camera_yaw_rad": physical.camera_yaw_rad,
        },
        "wheel_odometry": {
            "wheel_radius_m": physical.wheel_radius_m,
            "wheel_track_m": physical.wheel_track_m,
            "left_encoder_counts_per_revolution": (
                physical.left_encoder_counts_per_revolution
            ),
            "right_encoder_counts_per_revolution": (
                physical.right_encoder_counts_per_revolution
            ),
            "left_distance_scale": physical.left_distance_scale,
            "right_distance_scale": physical.right_distance_scale,
            "yaw_slip_scale": physical.yaw_slip_scale,
            "max_wheel_speed_mps": physical.max_wheel_speed_mps,
            "base_offset_x_m": physical.base_link_offset_x_m,
            "base_offset_y_m": physical.base_link_offset_y_m,
        },
        "fused_odometry": {
            "wheel_base_offset_x_m": physical.base_link_offset_x_m,
            "max_wheel_speed_mps": physical.max_wheel_speed_mps,
        },
        "waypoint_navigation": {
            "tracking_point_offset_x": physical.tracking_point_x_m,
            "tracking_point_offset_y": physical.tracking_point_y_m,
            "obstacle_sensor_forward_offset": physical.camera_x_m,
            "vehicle_front_offset": physical.vehicle_front_offset_m,
            "vehicle_length_m": physical.length_m,
            "vehicle_width_m": physical.width_m,
            "base_offset_x_m": physical.base_link_offset_x_m,
            "base_offset_y_m": physical.base_link_offset_y_m,
        },
        "rectangle_primitive_controller": {
            "wheel_track_m": physical.wheel_track_m,
        },
    }
    try:
        return mappings[component].copy()
    except KeyError as error:
        raise ValueError(f"unknown vehicle-config component '{component}'") from error
