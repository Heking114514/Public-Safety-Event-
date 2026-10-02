"""Static camera geometry used by the odometry-independent line follower."""

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
class GroundGrid:
    """Metric bird's-eye raster with rows along forward distance."""

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
    return getattr(params, name, default)


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


def undistort_depth(depth_m: np.ndarray, camera: CameraGeometry) -> np.ndarray:
    distortion = camera.distortion
    if distortion is None:
        return depth_m
    values = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if values.size == 0 or not np.any(np.abs(values) > 1.0e-12):
        return depth_m
    height, width = depth_m.shape[:2]
    map_u, map_v = cv2.initUndistortRectifyMap(
        camera.matrix(),
        values,
        None,
        camera.matrix(),
        (int(width), int(height)),
        cv2.CV_32FC1,
    )
    return cv2.remap(
        depth_m,
        map_u,
        map_v,
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0.0,
    )


def ground_to_pixel(
    camera: CameraGeometry,
    x: float,
    y: float,
) -> Optional[tuple[float, float]]:
    if camera.fx <= 0.0 or camera.fy <= 0.0 or camera.camera_z <= 0.0:
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


def depth_pixels_to_base(
    camera: CameraGeometry,
    u: np.ndarray,
    v: np.ndarray,
    depth_m: np.ndarray,
) -> np.ndarray:
    """Project aligned color/depth pixels into the base frame.

    The RealSense aligned-depth image uses color pixel coordinates. Depth is
    the optical-frame z distance, not Euclidean range.
    """

    if camera.fx <= 0.0 or camera.fy <= 0.0:
        return np.empty((0, 3), dtype=np.float64)
    u_values = np.asarray(u, dtype=np.float64)
    v_values = np.asarray(v, dtype=np.float64)
    z_values = np.asarray(depth_m, dtype=np.float64)
    if u_values.shape != v_values.shape or u_values.shape != z_values.shape:
        raise ValueError("u, v and depth_m must have the same shape")
    if u_values.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    optical = np.stack(
        (
            (u_values - camera.cx) * z_values / camera.fx,
            (v_values - camera.cy) * z_values / camera.fy,
            z_values,
        ),
        axis=-1,
    )
    base_vectors = optical @ camera.rotation().T
    base_vectors += np.array(
        [camera.camera_x, camera.camera_y, camera.camera_z],
        dtype=np.float64,
    )
    return base_vectors.reshape((-1, 3))


def ground_roi_polygon(
    image_shape: tuple[int, ...],
    camera: CameraGeometry,
    params: Any,
) -> list[tuple[int, int]]:
    height, width = image_shape[:2]
    min_x = max(
        float(
            parameter(
                params,
                "min_ground_x_m",
                parameter(params, "min_ground_x", 0.08),
            )
        ),
        camera.camera_x + 1.0e-3,
    )
    max_x = float(
        parameter(
            params,
            "max_ground_x_m",
            parameter(params, "max_ground_x", 1.60),
        )
    )
    max_y = float(
        parameter(
            params,
            "max_abs_ground_y_m",
            parameter(params, "max_abs_ground_y", 0.90),
        )
    )
    corners = [
        (max_x, -max_y),
        (max_x, max_y),
        (min_x, max_y),
        (min_x, -max_y),
    ]
    projected = [ground_to_pixel(camera, x, y) for x, y in corners]
    if any(point is None for point in projected):
        return []
    return [
        (
            int(round(max(-2.0 * width, min(3.0 * width, point[0])))),
            int(round(max(-2.0 * height, min(3.0 * height, point[1])))),
        )
        for point in projected
        if point is not None
    ]


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
    """Build ground-to-image maps for a fixed metric bird's-eye grid."""

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
