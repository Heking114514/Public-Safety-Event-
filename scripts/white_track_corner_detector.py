#!/usr/bin/env python3
"""RGB segmentation and ground projection for the offline white-track detector."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

VISION_SOURCE = Path(__file__).resolve().parents[1] / "src" / "vision_correction"
if str(VISION_SOURCE) not in sys.path:
    sys.path.insert(0, str(VISION_SOURCE))

from vision_correction import road_detector
from white_track_corner_topology import classify_ground_mask
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

__all__ = [
    "CROSS_JUNCTION",
    "LEFT_CORNER",
    "LOST",
    "RIGHT_CORNER",
    "STRAIGHT",
    "T_JUNCTION",
    "UNKNOWN",
    "WhiteTrackDetection",
    "WhiteTrackParameters",
    "classify_ground_mask",
    "detect_white_track",
    "segment_white_track",
]


def _rgb_features(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, np.ndarray, float, float, float, list[tuple[int, int]]]:
    roi, polygon = road_detector.ground_roi_mask(bgr.shape, camera, params)
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
        & (green_excess >= float(value(params, "green_excess_threshold", 8.0)))
        & (rgb[:, :, 1] >= float(value(params, "min_green_channel", 25.0)))
    )
    green_values = mean[green]
    green_baseline = float(np.median(green_values)) if green_values.size else 0.0
    white_threshold = float(
        np.clip(
            green_baseline + float(value(params, "white_margin_over_green", 28.0)),
            float(value(params, "min_white_value", 18.0)),
            float(value(params, "max_white_value", 175.0)),
        )
    )
    white_neutral = (
        (green_ratio <= float(value(params, "max_white_green_ratio", 0.365)))
        & (
            green_opponent
            <= float(value(params, "max_white_green_opponent", 0.090))
        )
    )
    min_lab_a = float(value(params, "min_white_lab_a", 122.0))
    if min_lab_a > 0.0:
        lab_a = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[:, :, 1]
        white_neutral &= lab_a >= min_lab_a
    white = (
        valid
        & (minimum >= white_threshold)
        & (spread <= float(value(params, "max_white_spread", 55.0)))
        & (
            green_excess
            <= float(value(params, "max_white_green_excess", 14.0))
        )
        & white_neutral
    )
    return (
        white.astype(np.uint8) * 255,
        green.astype(np.uint8) * 255,
        white_threshold,
        green_baseline,
        float(np.count_nonzero(green)) / max(1, int(np.count_nonzero(valid))),
        polygon,
    )


def segment_white_track(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: Any = WhiteTrackParameters(),
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    road_detector.GroundGrid,
    float,
    float,
    float,
    list[tuple[int, int]],
]:
    """Return image masks, projected ground masks and RGB statistics."""

    (
        white_image,
        green_image,
        threshold,
        baseline,
        image_green_fraction,
        polygon,
    ) = _rgb_features(bgr, camera, params)
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
    close_size = int(value(params, "ground_close_size", 5))
    open_size = int(value(params, "ground_open_size", 3))
    green_open_size = int(value(params, "green_open_size", 3))
    if close_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_CLOSE,
            kernel(close_size),
        )
    if open_size > 1:
        ground_white = cv2.morphologyEx(
            ground_white,
            cv2.MORPH_OPEN,
            kernel(open_size),
        )
    if green_open_size > 1:
        ground_green = cv2.morphologyEx(
            ground_green,
            cv2.MORPH_OPEN,
            kernel(green_open_size),
        )
    return (
        white_image,
        ground_green,
        ground_white,
        grid,
        threshold,
        baseline,
        image_green_fraction,
        polygon,
    )


def detect_white_track(
    bgr: np.ndarray,
    camera: road_detector.CameraGeometry,
    params: Any = WhiteTrackParameters(),
) -> WhiteTrackDetection:
    """Segment an RGB image and classify its local white-road topology."""

    (
        white_image,
        ground_green,
        ground_white,
        grid,
        threshold,
        baseline,
        _image_green_fraction,
        _polygon,
    ) = segment_white_track(bgr, camera, params)
    ground_green_fraction = float(np.mean(ground_green != 0))
    return classify_ground_mask(
        ground_white,
        ground_green,
        grid,
        params,
        white_threshold=threshold,
        green_baseline=baseline,
        green_fraction=ground_green_fraction,
        image_mask=white_image,
    )
