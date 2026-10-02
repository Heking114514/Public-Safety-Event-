"""Odometry-independent white-track observation.

The tracker estimates the road centre and tangent in the camera/base ground
plane. It never uses a pose, a map or a temporal odometry source.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from . import geometry as road_detector


TRACK = "TRACK"
LEFT_BRANCH = "LEFT_BRANCH"
RIGHT_BRANCH = "RIGHT_BRANCH"
JUNCTION = "JUNCTION"
UNSAFE_EDGE = "UNSAFE_EDGE"
LOST = "LOST"


@dataclass(frozen=True)
class TrackerConfig:
    min_ground_x_m: float = 0.12
    max_ground_x_m: float = 1.25
    max_abs_ground_y_m: float = 0.70
    ground_grid_resolution_m: float = 0.01
    roi_top_fraction: float = 0.0
    roi_bottom_fraction: float = 1.0
    control_min_x_m: float = 0.20
    control_max_x_m: float = 0.65
    control_reference_x_m: float = 0.30
    control_max_road_span_m: float = 0.20
    heading_min_x_m: float = 0.22
    heading_max_x_m: float = 0.55
    control_max_abs_y_m: float = 0.36
    green_excess_threshold: float = 8.0
    min_green_channel: float = 25.0
    white_margin_over_green: float = 28.0
    min_white_value: float = 18.0
    max_white_value: float = 175.0
    max_white_spread: float = 55.0
    max_white_green_excess: float = 14.0
    max_white_green_ratio: float = 0.365
    max_white_green_opponent: float = 0.090
    min_white_lab_a: float = 122.0
    ground_close_size: int = 5
    ground_open_size: int = 3
    green_open_size: int = 3
    green_dilate_size: int = 9
    min_ground_green_fraction: float = 0.08
    min_component_area: int = 60
    min_control_pixels: int = 24
    min_control_rows: int = 8
    max_center_jump_m: float = 0.30
    branch_span_m: float = 0.24
    branch_side_extent_m: float = 0.12
    heading_branch_exclusion_span_m: float = 0.22
    road_width_m: float = 0.20
    vehicle_width_m: float = 0.1535
    road_edge_margin_m: float = 0.010
    min_observed_road_width_m: float = 0.12
    min_recovery_road_width_m: float = 0.12
    max_drive_lateral_error_m: float = 0.09
    max_recovery_lateral_error_m: float = 0.12
    lateral_error_bias_m: float = 0.0
    use_depth_projection: bool = True
    depth_min_m: float = 0.12
    depth_max_m: float = 1.60
    depth_pixel_stride: int = 2
    depth_max_ground_abs_z_m: float = 0.035
    depth_min_control_pixels: int = 24
    depth_min_control_rows: int = 6
    depth_raster_radius_px: int = 1
    depth_component_dilate_size: int = 5
    depth_use_on_junction: bool = False
    depth_max_rgb_lateral_delta_m: float = 0.055
    depth_max_accepted_lateral_m: float = 0.085
    depth_max_accepted_road_width_m: float = 0.24


@dataclass(frozen=True)
class TrackObservation:
    state: str
    valid: bool
    confidence: float
    lateral_error_m: float
    heading_error_rad: float
    green_fraction: float
    component_area: int
    control_pixels: int
    control_rows: int
    mask: np.ndarray
    ground_mask: np.ndarray
    component_mask: np.ndarray
    grid: road_detector.GroundGrid
    road_left_y_m: float = 0.0
    road_right_y_m: float = 0.0
    road_clearance_m: float = 0.0
    lateral_source: str = "rgb_ground"
    depth_lateral_error_m: float = 0.0
    depth_control_pixels: int = 0
    depth_control_rows: int = 0
    depth_valid_fraction: float = 0.0
    depth_reject_reason: str = ""
    # A recoverable observation is deliberately not drive-valid: it may only
    # produce an in-place correction, never forward motion.
    can_recover: bool = False


def _parameter(params: Any, name: str, default: Any) -> Any:
    if isinstance(params, dict):
        return params.get(name, default)
    return getattr(params, name, default)


def _kernel(size: int) -> np.ndarray:
    value = max(1, int(size) | 1)
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (value, value))


def _empty_observation(
    image_mask: np.ndarray,
    ground_mask: np.ndarray,
    grid: road_detector.GroundGrid,
    green_fraction: float = 0.0,
) -> TrackObservation:
    return TrackObservation(
        state=LOST,
        valid=False,
        confidence=0.0,
        lateral_error_m=0.0,
        heading_error_rad=0.0,
        green_fraction=green_fraction,
        component_area=0,
        control_pixels=0,
        control_rows=0,
        mask=image_mask,
        ground_mask=ground_mask,
        component_mask=np.zeros_like(ground_mask),
        grid=grid,
        road_left_y_m=0.0,
        road_right_y_m=0.0,
        road_clearance_m=0.0,
    )


def _unsafe_observation(
    image_mask: np.ndarray,
    ground_mask: np.ndarray,
    component_mask: np.ndarray,
    grid: road_detector.GroundGrid,
    green_fraction: float,
    area: int,
    control_pixels: int,
    control_rows: int,
    lateral: float,
    road_left: float,
    road_right: float,
    road_width: float,
    params: Any,
    lateral_source: str = "rgb_ground",
    depth_lateral: float = 0.0,
    depth_control_pixels: int = 0,
    depth_control_rows: int = 0,
    depth_valid_fraction: float = 0.0,
    depth_reject_reason: str = "",
) -> TrackObservation:
    can_recover = (
        math.isfinite(lateral)
        and math.isfinite(road_left)
        and math.isfinite(road_right)
        and road_right > road_left
        and abs(road_left) > 1.0e-9
        and abs(road_right) > 1.0e-9
        and road_width
        >= float(_parameter(params, "min_recovery_road_width_m", 0.12))
        and abs(lateral)
        <= float(
            _parameter(params, "max_recovery_lateral_error_m", 0.12)
        )
    )
    return TrackObservation(
        state=UNSAFE_EDGE,
        valid=False,
        confidence=0.10,
        lateral_error_m=lateral,
        heading_error_rad=0.0,
        green_fraction=green_fraction,
        component_area=area,
        control_pixels=control_pixels,
        control_rows=control_rows,
        mask=image_mask,
        ground_mask=ground_mask,
        component_mask=component_mask,
        grid=grid,
        road_left_y_m=road_left,
        road_right_y_m=road_right,
        road_clearance_m=road_width,
        lateral_source=lateral_source,
        depth_lateral_error_m=depth_lateral,
        depth_control_pixels=depth_control_pixels,
        depth_control_rows=depth_control_rows,
        depth_valid_fraction=depth_valid_fraction,
        depth_reject_reason=depth_reject_reason,
        can_recover=can_recover,
    )


def _image_masks(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, np.ndarray, float]:
    roi, _ = road_detector.ground_roi_mask(bgr.shape, camera, params)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
    minimum = np.min(rgb, axis=2)
    maximum = np.max(rgb, axis=2)
    mean = np.mean(rgb, axis=2)
    spread = maximum - minimum
    green_excess = rgb[:, :, 1] - np.maximum(rgb[:, :, 0], rgb[:, :, 2])
    rgb_sum = np.maximum(np.sum(rgb, axis=2), 1.0)
    green_ratio = rgb[:, :, 1] / rgb_sum
    green_opponent = (
        2.0 * rgb[:, :, 1] - rgb[:, :, 0] - rgb[:, :, 2]
    ) / rgb_sum
    valid = roi != 0

    green = (
        valid
        & (
            green_excess
            >= float(_parameter(params, "green_excess_threshold", 8.0))
        )
        & (
            rgb[:, :, 1]
            >= float(_parameter(params, "min_green_channel", 25.0))
        )
    )
    green_values = mean[green]
    green_baseline = float(np.median(green_values)) if green_values.size else 0.0
    white_threshold = float(
        np.clip(
            green_baseline
            + float(_parameter(params, "white_margin_over_green", 28.0)),
            float(_parameter(params, "min_white_value", 18.0)),
            float(_parameter(params, "max_white_value", 175.0)),
        )
    )
    white_neutral = (
        (
            green_ratio
            <= float(_parameter(params, "max_white_green_ratio", 0.365))
        )
        & (
            green_opponent
            <= float(_parameter(params, "max_white_green_opponent", 0.090))
        )
    )
    min_lab_a = float(_parameter(params, "min_white_lab_a", 122.0))
    if min_lab_a > 0.0:
        lab_a = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[:, :, 1]
        white_neutral &= lab_a >= min_lab_a
    white = (
        valid
        & (minimum >= white_threshold)
        & (
            spread
            <= float(_parameter(params, "max_white_spread", 55.0))
        )
        & (
            green_excess
            <= float(_parameter(params, "max_white_green_excess", 14.0))
        )
        & white_neutral
    )
    return white.astype(np.uint8) * 255, green.astype(np.uint8) * 255, green_baseline


def _ground_masks(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, np.ndarray, road_detector.GroundGrid, float]:
    white_image, green_image, _ = _image_masks(bgr, camera, params)
    ground_white, grid = road_detector.project_mask_to_ground(
        white_image,
        camera,
        params,
    )
    ground_green, _ = road_detector.project_mask_to_ground(
        green_image,
        camera,
        params,
    )
    close_size = int(_parameter(params, "ground_close_size", 5))
    open_size = int(_parameter(params, "ground_open_size", 3))
    green_open_size = int(_parameter(params, "green_open_size", 3))
    if close_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_CLOSE,
            _kernel(close_size),
        )
    if open_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_OPEN,
            _kernel(open_size),
        )
    if green_open_size > 1:
        ground_green = cv2.morphologyEx(
            ground_green,
            cv2.MORPH_OPEN,
            _kernel(green_open_size),
        )
    green_fraction = float(np.mean(ground_green != 0))
    return ground_white, ground_green, grid, green_fraction


def _row_centres(
    component: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any,
) -> list[tuple[float, float, float, float, float]]:
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    minimum = float(_parameter(params, "heading_min_x_m", 0.22))
    maximum = float(_parameter(params, "heading_max_x_m", 0.85))
    rows: list[tuple[float, float, float, float, float]] = []
    for row in range(component.shape[0]):
        x = grid.x_min + (row + 0.5) * grid.resolution
        if x < minimum or x > maximum:
            continue
        columns = np.flatnonzero(component[row] != 0)
        if columns.size == 0:
            continue
        rows.append(
            (
                x,
                float(np.median(y_values[columns])),
                float(y_values[columns[0]]),
                float(y_values[columns[-1]]),
                float(y_values[columns[-1]] - y_values[columns[0]]),
            )
        )
    return rows


def _estimate_heading(
    rows: list[tuple[float, float, float, float, float]],
    params: Any,
) -> float:
    """Estimate the forward tangent without fitting across a crossing."""

    if len(rows) < 2:
        return 0.0
    branch_span = float(
        _parameter(
            params,
            "heading_branch_exclusion_span_m",
            _parameter(params, "branch_span_m", 0.42),
        )
    )
    narrow_rows = [row for row in rows if row[4] < branch_span]
    # Wide rows are the horizontal branch at a crossroad, not the forward
    # road tangent. Keep them only when there is not enough trunk left to fit.
    fit_rows = narrow_rows if len(narrow_rows) >= 3 else rows
    x = np.asarray([row[0] for row in fit_rows], dtype=np.float64)
    y = np.asarray([row[1] for row in fit_rows], dtype=np.float64)
    slope, _ = np.polyfit(x, y, 1)
    return math.atan(float(slope))


def _control_geometry(
    component: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any,
) -> tuple[float, float, float, float, int, int]:
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    min_x = float(_parameter(params, "control_min_x_m", 0.16))
    max_x = float(_parameter(params, "control_max_x_m", 0.55))
    max_y = float(_parameter(params, "control_max_abs_y_m", 0.52))
    allowed = np.abs(y_values) <= max_y
    rows: list[tuple[float, float, float, float]] = []
    control_pixels = 0
    control_rows = 0
    for row in range(component.shape[0]):
        x = grid.x_min + (row + 0.5) * grid.resolution
        if x < min_x or x > max_x:
            continue
        columns = np.flatnonzero((component[row] != 0) & allowed)
        if columns.size == 0:
            continue
        control_rows += 1
        control_pixels += int(columns.size)
        rows.append(
            (
                x,
                float(np.median(y_values[columns])),
                float(y_values[columns[0]]),
                float(y_values[columns[-1]]),
            )
        )
    if not rows:
        return 0.0, 0.0, 0.0, 0.0, control_pixels, control_rows

    # A crossing can make one connected component span most of the image.
    # Wide rows are not a usable estimate of the current lane centre. Keep
    # the narrow forward trunk and evaluate it at a fixed near-field distance
    # so a far horizontal branch cannot pull the car off the road.
    max_span = float(_parameter(params, "control_max_road_span_m", 0.32))
    trunk = [row for row in rows if row[3] - row[2] <= max_span]
    fit_rows = trunk if len(trunk) >= 3 else rows
    reference_x = float(
        _parameter(params, "control_reference_x_m", 0.24)
    )
    x = np.asarray([row[0] for row in fit_rows], dtype=np.float64)
    y = np.asarray([row[1] for row in fit_rows], dtype=np.float64)
    if len(fit_rows) >= 2 and np.ptp(x) > 1.0e-6:
        slope, intercept = np.polyfit(x, y, 1)
        lateral = float(slope * reference_x + intercept)
    else:
        lateral = min(
            fit_rows,
            key=lambda row: abs(row[0] - reference_x),
        )[1]
    nearest = min(fit_rows, key=lambda row: abs(row[0] - reference_x))
    return (
        lateral,
        nearest[2],
        nearest[3],
        nearest[3] - nearest[2],
        control_pixels,
        control_rows,
    )


def _apply_lateral_bias(
    lateral: float,
    road_left: float,
    road_right: float,
    params: Any,
) -> tuple[float, float, float]:
    bias = float(_parameter(params, "lateral_error_bias_m", 0.0))
    if not math.isfinite(bias) or abs(bias) <= 1.0e-12:
        return lateral, road_left, road_right
    return lateral + bias, road_left + bias, road_right + bias


def _depth_projected_ground_mask(
    white_image: np.ndarray,
    depth_m: np.ndarray,
    camera: road_detector.CameraGeometry,
    grid: road_detector.GroundGrid,
    params: Any,
    component_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, int, float]:
    if (
        depth_m is None
        or grid.width <= 0
        or grid.height <= 0
        or depth_m.shape[:2] != white_image.shape[:2]
    ):
        return np.zeros((grid.height, grid.width), dtype=np.uint8), 0, 0.0

    stride = max(1, int(_parameter(params, "depth_pixel_stride", 2)))
    sampled_white = white_image[::stride, ::stride] != 0
    rows, columns = np.nonzero(sampled_white)
    if rows.size == 0:
        return np.zeros((grid.height, grid.width), dtype=np.uint8), 0, 0.0

    v = rows.astype(np.float64) * stride
    u = columns.astype(np.float64) * stride
    depths = np.asarray(depth_m, dtype=np.float32)[
        rows.astype(np.int64) * stride,
        columns.astype(np.int64) * stride,
    ]
    minimum_depth = float(_parameter(params, "depth_min_m", 0.12))
    maximum_depth = float(_parameter(params, "depth_max_m", 1.60))
    valid_depth = (
        np.isfinite(depths)
        & (depths >= np.float32(minimum_depth))
        & (depths <= np.float32(maximum_depth))
    )
    candidate_count = int(rows.size)
    if not np.any(valid_depth):
        return np.zeros((grid.height, grid.width), dtype=np.uint8), 0, 0.0

    points = road_detector.depth_pixels_to_base(
        camera,
        u[valid_depth],
        v[valid_depth],
        depths[valid_depth],
    )
    if points.size == 0:
        return np.zeros((grid.height, grid.width), dtype=np.uint8), 0, 0.0

    min_x = float(_parameter(params, "min_ground_x_m", 0.12))
    max_x = float(_parameter(params, "max_ground_x_m", 1.25))
    max_y = float(_parameter(params, "control_max_abs_y_m", 0.36))
    max_ground_z = float(_parameter(params, "depth_max_ground_abs_z_m", 0.035))
    keep = (
        np.isfinite(points).all(axis=1)
        & (points[:, 0] >= min_x)
        & (points[:, 0] <= max_x)
        & (np.abs(points[:, 1]) <= max_y)
        & (np.abs(points[:, 2]) <= max_ground_z)
    )
    if not np.any(keep):
        return (
            np.zeros((grid.height, grid.width), dtype=np.uint8),
            0,
            float(np.count_nonzero(valid_depth) / max(1, candidate_count)),
        )

    selected = points[keep]
    grid_rows = np.floor((selected[:, 0] - grid.x_min) / grid.resolution).astype(
        np.int64
    )
    grid_columns = np.floor(
        (selected[:, 1] - grid.y_min) / grid.resolution
    ).astype(np.int64)
    inside = (
        (grid_rows >= 0)
        & (grid_rows < grid.height)
        & (grid_columns >= 0)
        & (grid_columns < grid.width)
    )
    ground = np.zeros((grid.height, grid.width), dtype=np.uint8)
    if not np.any(inside):
        return (
            ground,
            0,
            float(np.count_nonzero(valid_depth) / max(1, candidate_count)),
        )

    radius = max(0, int(_parameter(params, "depth_raster_radius_px", 1)))
    for row, column in zip(grid_rows[inside], grid_columns[inside]):
        if radius > 0:
            cv2.circle(ground, (int(column), int(row)), radius, 255, -1)
        else:
            ground[int(row), int(column)] = 255
    if component_mask is not None and component_mask.shape == ground.shape:
        dilate_size = int(_parameter(params, "depth_component_dilate_size", 5))
        if dilate_size > 1:
            component_support = cv2.dilate(
                component_mask,
                _kernel(dilate_size),
            )
        else:
            component_support = component_mask
        ground = cv2.bitwise_and(ground, component_support)
    return (
        ground,
        int(np.count_nonzero(ground)),
        float(np.count_nonzero(valid_depth) / max(1, candidate_count)),
    )


def _depth_control_geometry(
    white_image: np.ndarray,
    depth_m: np.ndarray | None,
    camera: road_detector.CameraGeometry,
    grid: road_detector.GroundGrid,
    params: Any,
    component_mask: np.ndarray | None = None,
) -> tuple[float, float, float, float, int, int, float] | None:
    if not bool(_parameter(params, "use_depth_projection", False)):
        return None
    if depth_m is None:
        return None
    depth_ground, _, valid_fraction = _depth_projected_ground_mask(
        white_image,
        depth_m,
        camera,
        grid,
        params,
        component_mask=component_mask,
    )
    if depth_ground.size == 0:
        return None
    close_size = int(_parameter(params, "ground_close_size", 5))
    open_size = int(_parameter(params, "ground_open_size", 3))
    if close_size > 1:
        depth_ground = cv2.morphologyEx(
            depth_ground,
            cv2.MORPH_CLOSE,
            _kernel(close_size),
        )
    if open_size > 1:
        depth_ground = cv2.morphologyEx(
            depth_ground,
            cv2.MORPH_OPEN,
            _kernel(open_size),
        )
    (
        lateral,
        road_left,
        road_right,
        observed_width,
        control_pixels,
        control_rows,
    ) = _control_geometry(depth_ground, grid, params)
    if control_pixels < int(_parameter(params, "depth_min_control_pixels", 24)):
        return None
    if control_rows < int(_parameter(params, "depth_min_control_rows", 6)):
        return None
    if not math.isfinite(lateral):
        return None
    return (
        lateral,
        road_left,
        road_right,
        observed_width,
        control_pixels,
        control_rows,
        valid_fraction,
    )


def _depth_reject_reason(
    rgb_lateral: float,
    rgb_width: float,
    state: str,
    depth_lateral: float,
    depth_width: float,
    params: Any,
) -> str:
    if state == JUNCTION and not bool(
        _parameter(params, "depth_use_on_junction", False)
    ):
        return "junction"
    if not all(
        math.isfinite(value)
        for value in (rgb_lateral, rgb_width, depth_lateral, depth_width)
    ):
        return "nonfinite"
    minimum_width = float(_parameter(params, "min_observed_road_width_m", 0.12))
    if depth_width < minimum_width:
        return "narrow_depth_width"
    max_width = float(_parameter(params, "depth_max_accepted_road_width_m", 0.24))
    if max_width > 0.0 and depth_width > max_width:
        return "wide_depth_width"
    max_lateral = float(_parameter(params, "depth_max_accepted_lateral_m", 0.085))
    if max_lateral > 0.0 and abs(depth_lateral) > max_lateral:
        return "large_depth_lateral"
    max_delta = float(_parameter(params, "depth_max_rgb_lateral_delta_m", 0.055))
    if max_delta > 0.0 and abs(depth_lateral - rgb_lateral) > max_delta:
        return "rgb_depth_disagree"
    return ""


def _road_support_is_safe(
    lateral: float,
    observed_width: float,
    params: Any,
) -> bool:
    if observed_width < float(
        _parameter(params, "min_observed_road_width_m", 0.12)
    ):
        return False
    return abs(lateral) <= float(
        _parameter(params, "max_drive_lateral_error_m", 0.09)
    )


def _select_component(
    ground_white: np.ndarray,
    ground_green: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any,
) -> tuple[np.ndarray | None, int, int, int]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(ground_white, 8)
    if count <= 1:
        return None, 0, 0, 0
    min_x = float(_parameter(params, "control_min_x_m", 0.16))
    max_x = float(_parameter(params, "control_max_x_m", 0.55))
    min_row = max(0, int(math.floor((min_x - grid.x_min) / grid.resolution)))
    max_row = min(
        grid.height - 1,
        int(math.ceil((max_x - grid.x_min) / grid.resolution)),
    )
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    allowed = np.abs(y_values) <= float(
        _parameter(params, "control_max_abs_y_m", 0.52)
    )
    green_context = cv2.dilate(
        ground_green,
        _kernel(int(_parameter(params, "green_dilate_size", 9))),
    ) > 0
    best: tuple[float, int, int, int] | None = None
    for label in range(1, count):
        component = labels == label
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < int(_parameter(params, "min_component_area", 60)):
            continue
        control = component[min_row : max_row + 1, :]
        pixels = int(np.count_nonzero(control[:, allowed]))
        rows = int(np.count_nonzero(np.any(control[:, allowed], axis=1)))
        if pixels == 0:
            continue
        adjacency = float(np.mean(green_context[component]))
        score = 5.0 * pixels + 15.0 * rows + 0.15 * area + 80.0 * adjacency
        candidate = (score, label, pixels, rows)
        if best is None or candidate[0] > best[0]:
            best = candidate
    if best is None:
        return None, 0, 0, 0
    return labels == best[1], int(stats[best[1], cv2.CC_STAT_AREA]), best[2], best[3]


def _classify_branch(
    rows: list[tuple[float, float, float, float, float]],
    params: Any,
) -> str:
    if len(rows) < 2:
        return TRACK
    x = np.asarray([row[0] for row in rows], dtype=np.float64)
    y = np.asarray([row[1] for row in rows], dtype=np.float64)
    slope, intercept = np.polyfit(x, y, 1)
    wide = [
        row
        for row in rows
        if row[4] >= float(_parameter(params, "branch_span_m", 0.42))
    ]
    if not wide:
        return TRACK
    branch_x = float(np.median([row[0] for row in wide]))
    selected = [row for row in rows if abs(row[0] - branch_x) <= 0.08]
    limit = float(_parameter(params, "branch_side_extent_m", 0.18))
    left = 0.0
    right = 0.0
    for row in selected:
        center = slope * row[0] + intercept
        left = max(left, row[3] - center)
        right = max(right, center - row[2])
    if left >= limit and right >= limit:
        return JUNCTION
    if left >= limit:
        return LEFT_BRANCH
    if right >= limit:
        return RIGHT_BRANCH
    return TRACK


def observe(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: TrackerConfig,
    depth_m: np.ndarray | None = None,
) -> TrackObservation:
    """Extract a current-frame track observation with no temporal state."""

    white_image, green_image, _ = _image_masks(bgr, camera, params)
    ground_white, grid = road_detector.project_mask_to_ground(
        white_image,
        camera,
        params,
    )
    ground_green, _ = road_detector.project_mask_to_ground(
        green_image,
        camera,
        params,
    )
    close_size = int(params.ground_close_size)
    open_size = int(params.ground_open_size)
    if close_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_CLOSE,
            _kernel(close_size),
        )
    if open_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_OPEN,
            _kernel(open_size),
        )
    if params.green_open_size > 1:
        ground_green = cv2.morphologyEx(
            ground_green,
            cv2.MORPH_OPEN,
            _kernel(params.green_open_size),
        )
    green_fraction = float(np.mean(ground_green != 0))
    component, area, control_pixels, control_rows = _select_component(
        ground_white,
        ground_green,
        grid,
        params,
    )
    if (
        component is None
        or green_fraction < params.min_ground_green_fraction
        or control_pixels < params.min_control_pixels
        or control_rows < params.min_control_rows
    ):
        return _empty_observation(
            white_image,
            ground_white,
            grid,
            green_fraction,
        )

    component_u8 = component.astype(np.uint8) * 255
    (
        lateral,
        road_left,
        road_right,
        observed_width,
        _,
        _,
    ) = _control_geometry(component_u8, grid, params)
    rows = _row_centres(component_u8, grid, params)
    heading = _estimate_heading(rows, params)
    state = _classify_branch(rows, params)
    lateral_source = "rgb_ground"
    depth_reject_reason = ""
    depth_lateral = 0.0
    depth_control_pixels = 0
    depth_control_rows = 0
    depth_valid_fraction = 0.0
    depth_geometry = _depth_control_geometry(
        white_image,
        depth_m,
        camera,
        grid,
        params,
        component_mask=component_u8,
    )
    if depth_geometry is not None:
        (
            depth_lateral,
            depth_left,
            depth_right,
            depth_width,
            depth_control_pixels,
            depth_control_rows,
            depth_valid_fraction,
        ) = depth_geometry
        depth_reject_reason = _depth_reject_reason(
            lateral,
            observed_width,
            state,
            depth_lateral,
            depth_width,
            params,
        )
        if not depth_reject_reason:
            lateral = depth_lateral
            road_left = depth_left
            road_right = depth_right
            observed_width = depth_width
            lateral_source = "depth_aligned"
    lateral, road_left, road_right = _apply_lateral_bias(
        lateral,
        road_left,
        road_right,
        params,
    )
    if not _road_support_is_safe(lateral, observed_width, params):
        return _unsafe_observation(
            white_image,
            ground_white,
            component_u8,
            grid,
            green_fraction,
            area,
            control_pixels,
            control_rows,
            lateral,
            road_left,
            road_right,
            observed_width,
            params,
            lateral_source=lateral_source,
            depth_lateral=depth_lateral,
            depth_control_pixels=depth_control_pixels,
            depth_control_rows=depth_control_rows,
            depth_valid_fraction=depth_valid_fraction,
            depth_reject_reason=depth_reject_reason,
        )
    if abs(lateral) > params.max_center_jump_m:
        return TrackObservation(
            state=LOST,
            valid=False,
            confidence=0.25,
            lateral_error_m=lateral,
            heading_error_rad=0.0,
            green_fraction=green_fraction,
            component_area=area,
            control_pixels=control_pixels,
            control_rows=control_rows,
            mask=white_image,
            ground_mask=ground_white,
            component_mask=component_u8,
            grid=grid,
            road_left_y_m=road_left,
            road_right_y_m=road_right,
            road_clearance_m=observed_width,
            lateral_source=lateral_source,
            depth_lateral_error_m=depth_lateral,
            depth_control_pixels=depth_control_pixels,
            depth_control_rows=depth_control_rows,
            depth_valid_fraction=depth_valid_fraction,
            depth_reject_reason=depth_reject_reason,
        )

    confidence = float(
        np.clip(
            0.45 * min(1.0, control_pixels / 160.0)
            + 0.35 * min(1.0, control_rows / 24.0)
            + 0.20
            * min(1.0, green_fraction / max(params.min_ground_green_fraction, 0.01)),
            0.0,
            1.0,
        )
    )
    return TrackObservation(
        state=state,
        valid=True,
        confidence=confidence,
        lateral_error_m=lateral,
        heading_error_rad=heading,
        green_fraction=green_fraction,
        component_area=area,
        control_pixels=control_pixels,
        control_rows=control_rows,
        mask=white_image,
        ground_mask=ground_white,
        component_mask=component_u8,
        grid=grid,
        road_left_y_m=road_left,
        road_right_y_m=road_right,
        road_clearance_m=observed_width,
        lateral_source=lateral_source,
        depth_lateral_error_m=depth_lateral,
        depth_control_pixels=depth_control_pixels,
        depth_control_rows=depth_control_rows,
        depth_valid_fraction=depth_valid_fraction,
        depth_reject_reason=depth_reject_reason,
    )
