"""Shared types for the offline white-track detector."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


STRAIGHT = "STRAIGHT"
LEFT_CORNER = "LEFT_CORNER"
RIGHT_CORNER = "RIGHT_CORNER"
T_JUNCTION = "T_JUNCTION"
CROSS_JUNCTION = "CROSS_JUNCTION"
UNKNOWN = "UNKNOWN"
LOST = "LOST"


@dataclass(frozen=True)
class WhiteTrackParameters:
    min_ground_x_m: float = 0.10
    max_ground_x_m: float = 1.35
    max_abs_ground_y_m: float = 0.75
    ground_grid_resolution_m: float = 0.01
    roi_top_fraction: float = 0.0
    roi_bottom_fraction: float = 1.0
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
    min_ground_green_fraction: float = 0.10
    min_component_area: int = 80
    near_band_start_x_m: float = 0.95
    near_center_abs_y_m: float = 0.40
    max_near_center_offset_m: float = 0.18
    min_near_center_pixels: int = 35
    min_near_support_rows: int = 8
    lane_half_width_m: float = 0.13
    branch_span_m: float = 0.42
    branch_side_extent_m: float = 0.18
    branch_row_band_m: float = 0.08
    continuation_margin_m: float = 0.12
    min_continuation_rows: int = 5
    max_center_slope: float = 0.75


@dataclass(frozen=True)
class WhiteTrackDetection:
    state: str
    confidence: float
    white_threshold: float
    green_baseline: float
    green_fraction: float
    component_area: int
    near_center_pixels: int
    near_support_rows: int
    near_center_y: float
    heading_error_rad: float
    branch_x: float
    left_extent: float
    right_extent: float
    forward_continuation_rows: int
    mask: np.ndarray
    green_mask: np.ndarray
    component_mask: np.ndarray
    ground_mask: np.ndarray


def value(params: Any, name: str, default: Any) -> Any:
    if isinstance(params, dict):
        return params.get(name, default)
    return getattr(params, name, default)


def kernel(value_to_round: int) -> np.ndarray:
    size = max(1, int(value_to_round) | 1)
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
