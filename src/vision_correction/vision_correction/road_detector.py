"""RGB dark-line detection and ground-plane projection helpers.

The detector is deliberately independent of ROS. Images are rectified first,
then only pixels whose camera rays intersect the measured ground rectangle are
considered. This keeps walls and most distant scenery out of the line mask.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

import cv2
import numpy as np


DEFAULT_OPTICAL_TO_BASE = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(frozen=True)
class CameraGeometry:
    fx: float
    fy: float
    cx: float
    cy: float
    camera_x: float = 0.055
    camera_y: float = 0.0
    camera_z: float = 0.121
    distortion: Optional[np.ndarray] = None
    color_to_base: Optional[np.ndarray] = None

    def matrix(self) -> np.ndarray:
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def rotation(self) -> np.ndarray:
        if self.color_to_base is None:
            return DEFAULT_OPTICAL_TO_BASE
        rotation = np.asarray(self.color_to_base, dtype=np.float64)
        if rotation.shape != (3, 3):
            return DEFAULT_OPTICAL_TO_BASE
        return rotation


@dataclass(frozen=True)
class DetectedGroundSegment:
    image_line: tuple[int, int, int, int]
    ax: float
    ay: float
    bx: float
    by: float
    length: float
    yaw: float


@dataclass(frozen=True)
class GroundGrid:
    """Metric bird's-eye raster used for line extraction."""

    x_min: float
    y_min: float
    resolution: float
    width: int
    height: int

    def pixel_to_ground(self, u: float, v: float) -> tuple[float, float]:
        return (
            self.x_min + (float(v) + 0.5) * self.resolution,
            self.y_min + (float(u) + 0.5) * self.resolution,
        )


def parameter(params: Any, name: str, default: Any) -> Any:
    if isinstance(params, dict):
        return params.get(name, default)
    value = getattr(params, name, default)
    return value


def undistort_bgr(image: np.ndarray, camera: CameraGeometry) -> np.ndarray:
    distortion = camera.distortion
    if distortion is None:
        return image
    values = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if values.size == 0 or not np.any(np.abs(values) > 1.0e-12):
        return image
    return cv2.undistort(
        image,
        camera.matrix(),
        values,
        None,
        camera.matrix(),
    )


def project_pixel(
    camera: CameraGeometry,
    u: float,
    v: float,
    limits: dict[str, float],
) -> Optional[tuple[float, float]]:
    if camera.fx <= 0.0 or camera.fy <= 0.0 or camera.camera_z <= 0.0:
        return None
    # CameraInfo for the RGB stream is expressed in the ROS optical frame:
    # x=right, y=down, z=forward. The static TF rotation maps that ray into
    # base_link, whose convention is x=forward, y=left, z=up.
    ray_optical = np.array(
        [
            (u - camera.cx) / camera.fx,
            (v - camera.cy) / camera.fy,
            1.0,
        ],
        dtype=np.float64,
    )
    ray_base = camera.rotation() @ ray_optical
    min_ray_down_z = float(limits.get("min_ray_down_z", 0.01))
    if ray_base[2] >= -min_ray_down_z:
        return None
    scale = -camera.camera_z / ray_base[2]
    if scale <= 0.0 or not math.isfinite(scale):
        return None
    x = camera.camera_x + scale * ray_base[0]
    y = camera.camera_y + scale * ray_base[1]
    if not (
        float(limits.get("min_x", 0.0))
        <= x
        <= float(limits.get("max_x", float("inf")))
        and abs(y) <= float(limits.get("max_abs_y", float("inf")))
    ):
        return None
    return float(x), float(y)


def ground_to_pixel(
    camera: CameraGeometry,
    x: float,
    y: float,
) -> Optional[tuple[float, float]]:
    if camera.fx <= 0.0 or camera.fy <= 0.0:
        return None
    point_base = np.array(
        [
            x - camera.camera_x,
            y - camera.camera_y,
            -camera.camera_z,
        ],
        dtype=np.float64,
    )
    ray_optical = camera.rotation().T @ point_base
    if ray_optical[2] <= 1.0e-9:
        return None
    return (
        float(camera.cx + camera.fx * ray_optical[0] / ray_optical[2]),
        float(camera.cy + camera.fy * ray_optical[1] / ray_optical[2]),
    )


def ground_roi_polygon(
    image_shape: tuple[int, ...],
    camera: CameraGeometry,
    params: Any,
) -> list[tuple[int, int]]:
    height, width = image_shape[:2]
    min_x_value = parameter(params, "min_ground_x_m", None)
    if min_x_value is None:
        min_x_value = parameter(params, "min_ground_x", 0.08)
    max_x_value = parameter(params, "max_ground_x_m", None)
    if max_x_value is None:
        max_x_value = parameter(params, "max_ground_x", 1.60)
    max_y_value = parameter(params, "max_abs_ground_y_m", None)
    if max_y_value is None:
        max_y_value = parameter(params, "max_abs_ground_y", 0.90)
    # A ground point behind the optical center cannot be projected. Clamp the
    # near edge instead of invalidating the entire ROI when a caller supplies
    # an overly small lower bound.
    min_x = max(float(min_x_value), camera.camera_x + 1.0e-3)
    max_x = float(max_x_value)
    max_y = float(max_y_value)
    corners = [
        (max_x, -max_y),
        (max_x, max_y),
        (min_x, max_y),
        (min_x, -max_y),
    ]
    projected = [ground_to_pixel(camera, x, y) for x, y in corners]
    if any(point is None for point in projected):
        return []
    polygon = [
        (
            int(round(max(-2.0 * width, min(3.0 * width, point[0])))),
            int(round(max(-2.0 * height, min(3.0 * height, point[1])))),
        )
        for point in projected
        if point is not None
    ]
    return polygon


def ground_roi_mask(
    image_shape: tuple[int, ...],
    camera: CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, list[tuple[int, int]]]:
    height, width = image_shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = ground_roi_polygon(image_shape, camera, params)
    if polygon:
        cv2.fillConvexPoly(mask, np.asarray(polygon, dtype=np.int32), 255)

    top_fraction = float(parameter(params, "roi_top_fraction", 0.54))
    bottom_fraction = float(parameter(params, "roi_bottom_fraction", 1.0))
    top_fraction = max(0.0, min(1.0, top_fraction))
    bottom_fraction = max(top_fraction, min(1.0, bottom_fraction))
    top = max(0, min(height, int(round(height * top_fraction))))
    bottom = max(top, min(height, int(round(height * bottom_fraction))))
    mask[:top, :] = 0
    mask[bottom:, :] = 0
    return mask, polygon


def ground_grid(
    image_shape: tuple[int, ...],
    camera: CameraGeometry,
    params: Any,
) -> tuple[GroundGrid, np.ndarray, np.ndarray]:
    """Build ground-to-image maps for a fixed metric bird's-eye grid.

    The grid uses rows for forward distance ``x`` and columns for lateral
    distance ``y``. Invalid cells are filled with -1 and are ignored by
    ``cv2.remap``. Keeping this conversion in one place makes the runtime,
    offline correction and debug renderer use exactly the same geometry.
    """

    del image_shape
    min_x = float(
        parameter(params, "min_ground_x_m", parameter(params, "min_ground_x", 0.10))
    )
    max_x = float(
        parameter(params, "max_ground_x_m", parameter(params, "max_ground_x", 1.60))
    )
    max_abs_y = float(
        parameter(
            params,
            "max_abs_ground_y_m",
            parameter(params, "max_abs_ground_y", 0.65),
        )
    )
    resolution = max(
        0.002,
        float(parameter(params, "ground_grid_resolution_m", 0.01)),
    )
    if max_x <= min_x or max_abs_y <= 0.0:
        empty = GroundGrid(min_x, -max_abs_y, resolution, 0, 0)
        return empty, np.empty((0, 0), np.float32), np.empty((0, 0), np.float32)

    width = max(1, int(math.ceil(2.0 * max_abs_y / resolution)))
    height = max(1, int(math.ceil((max_x - min_x) / resolution)))
    grid = GroundGrid(min_x, -max_abs_y, resolution, width, height)

    y_values = grid.y_min + (np.arange(width, dtype=np.float64) + 0.5) * resolution
    x_values = grid.x_min + (np.arange(height, dtype=np.float64) + 0.5) * resolution
    x_grid, y_grid = np.meshgrid(x_values, y_values, indexing="ij")
    points_base = np.stack(
        (
            x_grid - camera.camera_x,
            y_grid - camera.camera_y,
            np.full_like(x_grid, -camera.camera_z),
        ),
        axis=-1,
    )
    # A row vector multiplied by R is equivalent to R.T @ point_base.
    rays_optical = points_base @ camera.rotation()
    valid = rays_optical[..., 2] > 1.0e-9
    map_u = np.full((height, width), -1.0, dtype=np.float32)
    map_v = np.full((height, width), -1.0, dtype=np.float32)
    map_u[valid] = (
        camera.cx
        + camera.fx * rays_optical[..., 0][valid] / rays_optical[..., 2][valid]
    ).astype(np.float32)
    map_v[valid] = (
        camera.cy
        + camera.fy * rays_optical[..., 1][valid] / rays_optical[..., 2][valid]
    ).astype(np.float32)
    return grid, map_u, map_v


def project_mask_to_ground(
    mask: np.ndarray,
    camera: CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, GroundGrid]:
    """Sample an image mask into a uniform metric ground-plane mask."""

    grid, map_u, map_v = ground_grid(mask.shape, camera, params)
    if grid.width <= 0 or grid.height <= 0:
        return np.zeros((0, 0), dtype=np.uint8), grid
    ground = cv2.remap(
        mask,
        map_u,
        map_v,
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    return ground, grid


def _row_thresholds(
    max_rgb: np.ndarray,
    roi: np.ndarray,
    params: Any,
) -> np.ndarray:
    height = max_rgb.shape[0]
    percentile = float(parameter(params, "rgb_dark_percentile", 72.0))
    margin = float(parameter(params, "rgb_dark_margin", 26.0))
    minimum = float(parameter(params, "rgb_dark_min_threshold", 24.0))
    maximum = float(parameter(params, "rgb_dark_max_threshold", 125.0))
    thresholds = np.full(height, minimum, dtype=np.float32)
    for row in range(height):
        values = max_rgb[row][roi[row] != 0]
        if values.size < 12:
            continue
        baseline = float(np.percentile(values, percentile))
        thresholds[row] = np.clip(baseline - margin, minimum, maximum)
    return thresholds[:, None]


def filter_components(mask: np.ndarray, params) -> np.ndarray:
    min_area = int(parameter(params, "min_component_area", 8))
    max_area_fraction = float(
        parameter(params, "max_component_area_fraction", 0.12)
    )
    min_width = int(parameter(params, "min_component_width", 3))
    min_height = int(parameter(params, "min_component_height", 2))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    output = np.zeros_like(mask)
    max_area = max_area_fraction * float(mask.shape[0] * mask.shape[1])
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        if area < min_area or area > max_area:
            continue
        if width < min_width or height < min_height:
            continue
        output[labels == label] = 255
    return output


def build_black_line_mask(
    bgr: np.ndarray,
    camera: CameraGeometry,
    params: Any,
) -> tuple[np.ndarray, np.ndarray, list[tuple[int, int]]]:
    """Return (mask, ground_roi, projected_roi_polygon)."""

    roi, polygon = ground_roi_mask(bgr.shape, camera, params)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    max_rgb = np.max(rgb, axis=2).astype(np.float32)
    min_rgb = np.min(rgb, axis=2).astype(np.float32)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    thresholds = _row_thresholds(max_rgb, roi, params)
    rgb_dark = max_rgb <= thresholds

    sigma = max(
        1.0,
        float(parameter(params, "local_dark_sigma", parameter(params, "local_sigma", 9.0))),
    )
    local_background = cv2.GaussianBlur(
        max_rgb,
        (0, 0),
        sigmaX=sigma,
        sigmaY=sigma,
    )
    local_delta = float(
        parameter(params, "local_dark_delta", parameter(params, "local_delta", 12.0))
    )
    local_dark = (local_background - max_rgb) >= local_delta

    blackhat_size = max(
        3,
        int(parameter(params, "blackhat_size", 25)) | 1,
    )
    blackhat_threshold = float(
        parameter(params, "blackhat_threshold", 10.0)
    )
    blackhat = cv2.morphologyEx(
        cv2.GaussianBlur(gray, (3, 3), 0),
        cv2.MORPH_BLACKHAT,
        cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (blackhat_size, blackhat_size),
        ),
    )
    line_response = blackhat >= blackhat_threshold

    max_color_spread = float(parameter(params, "max_color_spread", 90.0))
    nearly_black = (max_rgb - min_rgb) <= max_color_spread
    detector = str(parameter(params, "detector", "rgb_or_local"))
    if detector == "rgb_dark":
        response = rgb_dark
    elif detector == "local_dark":
        response = local_dark
    elif detector == "rgb_and_local":
        response = rgb_dark & local_dark
    elif detector == "blackhat":
        response = line_response
    else:
        # The default is deliberately conservative: a pixel must be RGB-dark
        # and have either local contrast or a line-shaped blackhat response.
        response = rgb_dark & (local_dark | line_response)

    mask = (roi.astype(bool) & nearly_black & response).astype(np.uint8) * 255

    morphology_size = max(
        1,
        int(parameter(params, "morphology_size", 3)),
    )
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (morphology_size, morphology_size),
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = filter_components(mask, params)
    return mask, roi, polygon


def _line_support_fraction(mask: np.ndarray, line: tuple[int, int, int, int]) -> float:
    x1, y1, x2, y2 = line
    length = max(abs(x2 - x1), abs(y2 - y1)) + 1
    xs = np.rint(np.linspace(x1, x2, length)).astype(np.int32)
    ys = np.rint(np.linspace(y1, y2, length)).astype(np.int32)
    valid = (
        (xs >= 0)
        & (xs < mask.shape[1])
        & (ys >= 0)
        & (ys < mask.shape[0])
    )
    if not np.any(valid):
        return 0.0
    return float(np.count_nonzero(mask[ys[valid], xs[valid]])) / float(
        np.count_nonzero(valid)
    )


def _ground_line_from_grid(
    grid: GroundGrid,
    line: tuple[int, int, int, int],
) -> tuple[float, float, float, float, float, float]:
    first = grid.pixel_to_ground(line[0], line[1])
    second = grid.pixel_to_ground(line[2], line[3])
    length = math.hypot(second[0] - first[0], second[1] - first[1])
    return (
        first[0],
        first[1],
        second[0],
        second[1],
        length,
        math.atan2(second[1] - first[1], second[0] - first[0]),
    )


def _merge_ground_segments(
    segments: list[DetectedGroundSegment],
    params: Any,
) -> list[DetectedGroundSegment]:
    """Merge Hough fragments that describe one physical ground line."""

    angle_limit = float(parameter(params, "line_merge_angle_rad", 0.14))
    distance_limit = float(parameter(params, "line_merge_distance_m", 0.045))
    gap_limit = float(parameter(params, "line_merge_gap_m", 0.18))

    def canonical(segment: DetectedGroundSegment):
        theta = segment.yaw % math.pi
        direction = np.array([math.cos(theta), math.sin(theta)])
        normal = np.array([-math.sin(theta), math.cos(theta)])
        first = np.array([segment.ax, segment.ay])
        second = np.array([segment.bx, segment.by])
        rho = float(normal @ (0.5 * (first + second)))
        return (
            theta,
            rho,
            min(float(direction @ first), float(direction @ second)),
            max(float(direction @ first), float(direction @ second)),
            segment.length,
        )

    def angle_error(first: float, second: float) -> float:
        value = (first - second + 0.5 * math.pi) % math.pi - 0.5 * math.pi
        return abs(value)

    def interval_gap(
        first_min: float,
        first_max: float,
        second_min: float,
        second_max: float,
    ) -> float:
        return max(first_min - second_max, second_min - first_max, 0.0)

    clusters: list[list[DetectedGroundSegment]] = []
    for segment in sorted(segments, key=lambda item: item.length, reverse=True):
        theta, rho, line_min, line_max, _ = canonical(segment)
        target = None
        target_score = float("inf")
        for index, cluster in enumerate(clusters):
            values = [canonical(item) for item in cluster]
            total = sum(item[4] for item in values)
            center_theta = (
                0.5
                * math.atan2(
                    sum(item[4] * math.sin(2.0 * item[0]) for item in values),
                    sum(item[4] * math.cos(2.0 * item[0]) for item in values),
                )
            ) % math.pi
            center_rho = sum(item[1] * item[4] for item in values) / total
            cluster_min = min(item[2] for item in values)
            cluster_max = max(item[3] for item in values)
            gap = interval_gap(line_min, line_max, cluster_min, cluster_max)
            if (
                angle_error(theta, center_theta) <= angle_limit
                and abs(rho - center_rho) <= distance_limit
                and gap <= gap_limit
            ):
                score = angle_error(theta, center_theta) + abs(rho - center_rho) + gap
                if score < target_score:
                    target = index
                    target_score = score
        if target is None:
            clusters.append([segment])
        else:
            clusters[target].append(segment)

    result: list[DetectedGroundSegment] = []
    for cluster in clusters:
        values = [canonical(item) for item in cluster]
        total = sum(item[4] for item in values)
        theta = (
            0.5
            * math.atan2(
                sum(item[4] * math.sin(2.0 * item[0]) for item in values),
                sum(item[4] * math.cos(2.0 * item[0]) for item in values),
            )
        ) % math.pi
        direction = np.array([math.cos(theta), math.sin(theta)])
        normal = np.array([-math.sin(theta), math.cos(theta)])
        rho = sum(item[1] * item[4] for item in values) / total
        projections = []
        for item in cluster:
            projections.extend(
                [
                    float(direction @ np.array([item.ax, item.ay])),
                    float(direction @ np.array([item.bx, item.by])),
                ]
            )
        first = normal * rho + direction * min(projections)
        second = normal * rho + direction * max(projections)
        length = float(np.linalg.norm(second - first))
        if length < float(parameter(params, "min_ground_segment_length_m", 0.06)):
            continue
        result.append(
            DetectedGroundSegment(
                cluster[0].image_line,
                float(first[0]),
                float(first[1]),
                float(second[0]),
                float(second[1]),
                length,
                theta,
            )
        )
    result.sort(key=lambda item: item.length, reverse=True)
    return result[: int(parameter(params, "max_segments", 45))]


def detect_ground_segments(
    bgr: np.ndarray,
    camera: CameraGeometry,
    params: Any,
) -> tuple[list[DetectedGroundSegment], np.ndarray, np.ndarray, list[tuple[int, int]]]:
    mask, roi, polygon = build_black_line_mask(bgr, camera, params)
    ground_mask, grid = project_mask_to_ground(mask, camera, params)
    if ground_mask.size == 0:
        return [], mask, roi, polygon
    ground_kernel_size = max(
        1,
        int(parameter(params, "ground_morphology_size_px", 3)),
    )
    if ground_kernel_size > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (ground_kernel_size, ground_kernel_size),
        )
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_OPEN, kernel)
        ground_mask = cv2.morphologyEx(ground_mask, cv2.MORPH_CLOSE, kernel)

    lines = cv2.HoughLinesP(
        ground_mask,
        1.0,
        np.pi / 180.0,
        int(parameter(params, "ground_hough_threshold", 10)),
        minLineLength=max(
            3,
            int(
                round(
                    float(
                        parameter(
                            params,
                            "min_ground_segment_length_m",
                            0.06,
                        )
                    )
                    / grid.resolution
                )
            ),
        ),
        maxLineGap=max(
            1,
            int(
                round(
                    float(parameter(params, "ground_max_line_gap_m", 0.10))
                    / grid.resolution
                )
            ),
        ),
    )
    if lines is None:
        return [], mask, roi, polygon

    result: list[DetectedGroundSegment] = []
    for raw in lines[:, 0, :]:
        line = tuple(int(value) for value in raw)
        support = _line_support_fraction(ground_mask, line)
        if support < float(parameter(params, "min_line_support_fraction", 0.50)):
            continue
        ax, ay, bx, by, length, yaw = _ground_line_from_grid(grid, line)
        if length < float(parameter(params, "min_ground_segment_length_m", 0.06)):
            continue
        first_pixel = ground_to_pixel(camera, ax, ay)
        second_pixel = ground_to_pixel(camera, bx, by)
        if first_pixel is None or second_pixel is None:
            continue
        result.append(
            DetectedGroundSegment(
                (
                    int(round(first_pixel[0])),
                    int(round(first_pixel[1])),
                    int(round(second_pixel[0])),
                    int(round(second_pixel[1])),
                ),
                ax,
                ay,
                bx,
                by,
                length,
                yaw,
            )
        )
    return _merge_ground_segments(result, params), mask, roi, polygon
