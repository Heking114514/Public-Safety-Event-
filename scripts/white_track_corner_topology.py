"""Ground-plane topology classification for the offline white-track detector."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

VISION_SOURCE = Path(__file__).resolve().parents[1] / "src" / "vision_correction"
if str(VISION_SOURCE) not in sys.path:
    sys.path.insert(0, str(VISION_SOURCE))

from vision_correction import road_detector
from white_track_corner_types import (
    CROSS_JUNCTION,
    LEFT_CORNER,
    LOST,
    RIGHT_CORNER,
    STRAIGHT,
    T_JUNCTION,
    UNKNOWN,
    WhiteTrackDetection,
    WhiteTrackParameters,
    kernel,
    value,
)


def _component_candidates(
    ground_mask: np.ndarray,
    green_mask: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any,
) -> tuple[np.ndarray | None, int, int, int, float]:
    count, labels, stats, _ = cv2.connectedComponentsWithStats(ground_mask, 8)
    if count <= 1:
        return None, 0, 0, 0, 0.0

    near_row = max(
        0,
        min(
            grid.height - 1,
            int(
                math.floor(
                    (float(value(params, "near_band_start_x_m", 0.95)) - grid.x_min)
                    / grid.resolution
                )
            ),
        ),
    )
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    near_center = np.abs(y_values) <= float(
        value(params, "near_center_abs_y_m", 0.40)
    )
    green_near = cv2.dilate(
        green_mask,
        kernel(int(value(params, "green_dilate_size", 9))),
    ) > 0
    best_label = 0
    best_score = -float("inf")
    best_values = (0, 0, 0.0)
    for label in range(1, count):
        component = labels == label
        area = int(np.count_nonzero(component))
        if area < int(value(params, "min_component_area", 80)):
            continue
        near = component[near_row:, :]
        near_center_pixels = int(np.count_nonzero(near[:, near_center]))
        near_support_rows = int(
            np.count_nonzero(np.any(near[:, near_center], axis=1))
        )
        adjacency = float(np.mean(green_near[component]))
        score = (
            5.0 * near_center_pixels
            + 12.0 * near_support_rows
            + 0.20 * area
            + 120.0 * adjacency
        )
        if score > best_score:
            best_score = score
            best_label = label
            best_values = (near_center_pixels, near_support_rows, adjacency)

    if best_label == 0:
        return None, 0, 0, 0, 0.0
    return (
        (labels == best_label).astype(np.uint8) * 255,
        int(stats[best_label, cv2.CC_STAT_AREA]),
        best_values[0],
        best_values[1],
        best_values[2],
    )


def _row_runs(
    component: np.ndarray,
    grid: road_detector.GroundGrid,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    for row in range(component.shape[0]):
        columns = np.flatnonzero(component[row] != 0)
        if columns.size == 0:
            continue
        rows.append(
            {
                "row": float(row),
                "x": grid.x_min + (row + 0.5) * grid.resolution,
                "min_y": float(y_values[columns[0]]),
                "max_y": float(y_values[columns[-1]]),
                "center_y": float(np.median(y_values[columns])),
                "span": float(y_values[columns[-1]] - y_values[columns[0]]),
                "count": float(columns.size),
            }
        )
    return rows


def _median_near_center(
    component: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any,
) -> tuple[float, int, int]:
    near_row = max(
        0,
        min(
            grid.height - 1,
            int(
                math.floor(
                    (float(value(params, "near_band_start_x_m", 0.95)) - grid.x_min)
                    / grid.resolution
                )
            ),
        ),
    )
    y_values = grid.y_min + (np.arange(grid.width) + 0.5) * grid.resolution
    allowed = np.abs(y_values) <= float(
        value(params, "near_center_abs_y_m", 0.40)
    )
    values: list[float] = []
    rows = 0
    for row in range(near_row, component.shape[0]):
        columns = np.flatnonzero((component[row] != 0) & allowed)
        if columns.size:
            rows += 1
            values.extend(y_values[columns].tolist())
    if not values:
        return 0.0, 0, rows
    return float(np.median(np.asarray(values))), len(values), rows


def _fit_near_center(
    rows: list[dict[str, float]],
    params: Any,
) -> tuple[float, float]:
    start = float(value(params, "near_band_start_x_m", 0.95))
    selected = [
        item
        for item in rows
        if item["x"] >= start
        and item["span"] <= float(value(params, "branch_span_m", 0.42))
    ]
    if len(selected) < 3:
        selected = [item for item in rows if item["x"] >= start]
    if len(selected) < 2:
        return 0.0, 0.0
    x = np.asarray([item["x"] for item in selected], dtype=np.float64)
    y = np.asarray([item["center_y"] for item in selected], dtype=np.float64)
    slope, intercept = np.polyfit(x, y, 1)
    maximum = float(value(params, "max_center_slope", 0.75))
    return float(np.clip(slope, -maximum, maximum)), float(intercept)


def _branch_measurement(
    rows: list[dict[str, float]],
    slope: float,
    intercept: float,
    params: Any,
) -> tuple[float, float, float]:
    branch_rows = [
        item
        for item in rows
        if item["span"] >= float(value(params, "branch_span_m", 0.42))
    ]
    if not branch_rows:
        return float("nan"), 0.0, 0.0
    branch_x = float(np.median([item["x"] for item in branch_rows]))
    band = float(value(params, "branch_row_band_m", 0.08))
    selected = [item for item in rows if abs(item["x"] - branch_x) <= band]
    if not selected:
        selected = branch_rows
    left_extent = 0.0
    right_extent = 0.0
    for item in selected:
        center = slope * item["x"] + intercept
        left_extent = max(left_extent, item["max_y"] - center)
        right_extent = max(right_extent, center - item["min_y"])
    return branch_x, left_extent, right_extent


def _continuation_rows(
    rows: list[dict[str, float]],
    branch_x: float,
    slope: float,
    intercept: float,
    params: Any,
) -> int:
    margin = float(value(params, "continuation_margin_m", 0.12))
    half_width = float(value(params, "lane_half_width_m", 0.13)) * 1.35
    cutoff = (
        branch_x
        if math.isfinite(branch_x)
        else float(value(params, "near_band_start_x_m", 0.95))
    )
    count = 0
    for item in rows:
        if item["x"] >= cutoff - margin:
            continue
        expected = slope * item["x"] + intercept
        if (
            item["min_y"] <= expected + half_width
            and item["max_y"] >= expected - half_width
        ):
            count += 1
    return count


def _confidence(
    near_pixels: int,
    near_rows: int,
    green_fraction: float,
    branch_x: float,
    left_extent: float,
    right_extent: float,
    continuation: int,
    params: Any,
) -> float:
    near_score = min(
        1.0,
        near_pixels
        / max(1.0, 4.0 * float(value(params, "min_near_center_pixels", 35))),
    )
    row_score = min(
        1.0,
        near_rows
        / max(1.0, 2.0 * float(value(params, "min_near_support_rows", 8))),
    )
    green_score = min(
        1.0,
        green_fraction
        / max(0.01, float(value(params, "min_ground_green_fraction", 0.10))),
    )
    shape_score = 0.35 if math.isfinite(branch_x) else 0.15
    side_score = min(
        1.0,
        max(left_extent, right_extent)
        / max(0.01, float(value(params, "branch_side_extent_m", 0.18))),
    )
    continuation_score = min(
        1.0,
        continuation
        / max(1.0, float(value(params, "min_continuation_rows", 5))),
    )
    return float(
        np.clip(
            0.30 * near_score
            + 0.20 * row_score
            + 0.20 * green_score
            + 0.15 * shape_score
            + 0.10 * side_score
            + 0.05 * continuation_score,
            0.0,
            1.0,
        )
    )


def classify_ground_mask(
    ground_mask: np.ndarray,
    green_mask: np.ndarray,
    grid: road_detector.GroundGrid,
    params: Any = WhiteTrackParameters(),
    *,
    white_threshold: float = 0.0,
    green_baseline: float = 0.0,
    green_fraction: float | None = None,
    image_mask: np.ndarray | None = None,
) -> WhiteTrackDetection:
    """Classify a projected white-road mask."""

    if ground_mask.size == 0 or grid.width <= 0 or grid.height <= 0:
        empty = np.zeros_like(ground_mask)
        return WhiteTrackDetection(
            LOST,
            0.0,
            white_threshold,
            green_baseline,
            0.0,
            0,
            0,
            0,
            0.0,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0,
            image_mask if image_mask is not None else empty,
            empty,
            empty,
            empty,
        )

    if green_fraction is None:
        green_fraction = float(np.mean(green_mask != 0))
    component, area, near_pixels, near_rows, adjacency = _component_candidates(
        ground_mask,
        green_mask,
        grid,
        params,
    )
    empty = np.zeros_like(ground_mask)
    min_green = float(value(params, "min_ground_green_fraction", 0.10))
    near_center_y = 0.0
    if component is not None:
        near_center_y, _, _ = _median_near_center(component, grid, params)
    if (
        component is None
        or near_pixels < int(value(params, "min_near_center_pixels", 35))
        or near_rows < int(value(params, "min_near_support_rows", 8))
        or green_fraction < min_green
        or adjacency < 0.08
        or abs(near_center_y)
        > float(value(params, "max_near_center_offset_m", 0.18))
    ):
        return WhiteTrackDetection(
            LOST,
            _confidence(
                near_pixels,
                near_rows,
                float(green_fraction),
                float("nan"),
                0.0,
                0.0,
                0,
                params,
            ),
            white_threshold,
            green_baseline,
            float(green_fraction),
            area,
            near_pixels,
            near_rows,
            near_center_y,
            0.0,
            float("nan"),
            0.0,
            0.0,
            0,
            image_mask if image_mask is not None else empty,
            green_mask,
            component if component is not None else empty,
            ground_mask,
        )

    rows = _row_runs(component, grid)
    slope, intercept = _fit_near_center(rows, params)
    heading_error = math.atan(slope)
    branch_x, left_extent, right_extent = _branch_measurement(
        rows,
        slope,
        intercept,
        params,
    )
    continuation = _continuation_rows(
        rows,
        branch_x,
        slope,
        intercept,
        params,
    )
    confidence = _confidence(
        near_pixels,
        near_rows,
        float(green_fraction),
        branch_x,
        left_extent,
        right_extent,
        continuation,
        params,
    )

    branch_limit = float(value(params, "branch_side_extent_m", 0.18))
    has_left = left_extent >= branch_limit
    has_right = right_extent >= branch_limit
    if not math.isfinite(branch_x):
        state = STRAIGHT if continuation >= 2 else UNKNOWN
    elif has_left and has_right:
        state = (
            CROSS_JUNCTION
            if continuation >= int(value(params, "min_continuation_rows", 5))
            else T_JUNCTION
        )
    elif has_left:
        state = LEFT_CORNER
    elif has_right:
        state = RIGHT_CORNER
    else:
        state = STRAIGHT if continuation >= 2 else UNKNOWN

    return WhiteTrackDetection(
        state,
        confidence,
        white_threshold,
        green_baseline,
        float(green_fraction),
        area,
        near_pixels,
        near_rows,
        near_center_y,
        heading_error,
        branch_x,
        left_extent,
        right_extent,
        continuation,
        image_mask if image_mask is not None else empty,
        green_mask,
        component,
        ground_mask,
    )
