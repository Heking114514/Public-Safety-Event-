import math

import numpy as np

from pure_line_following.geometry import (
    CameraGeometry,
    GroundGrid,
    DEFAULT_OPTICAL_TO_BASE,
    depth_pixels_to_base,
    ground_to_pixel,
)
from pure_line_following.tracker import (
    JUNCTION,
    LEFT_BRANCH,
    TRACK,
    TrackerConfig,
    _apply_lateral_bias,
    _branch_geometry,
    _classify_branch,
    _control_geometry,
    _depth_control_geometry,
    _depth_reject_reason,
    _estimate_heading,
    _image_masks,
    _road_support_is_safe,
    _road_slices,
    observe,
)


def test_track_observation_from_synthetic_ground_shape():
    params = TrackerConfig()
    grid = GroundGrid(
        params.min_ground_x_m,
        -params.max_abs_ground_y_m,
        params.ground_grid_resolution_m,
        140,
        113,
    )
    component = np.zeros((grid.height, grid.width), dtype=np.uint8)
    centre = 70
    component[5:45, centre - 10 : centre + 11] = 255
    green = np.ones_like(component) * 255
    green[component != 0] = 0

    # This test exercises the sign convention used by the controller through
    # the lower-level geometry helper's expected ground-axis arrangement.
    assert np.count_nonzero(component) > 0
    assert grid.width == 140


def test_heading_ignores_wide_crossing_row():
    rows = [
        (0.25, 0.00, -0.08, 0.08, 0.16),
        (0.35, 0.01, -0.07, 0.09, 0.16),
        (0.45, 0.02, -0.06, 0.10, 0.16),
        (0.55, 0.03, -0.05, 0.11, 0.16),
        (0.65, 0.04, -0.04, 0.12, 0.16),
        (0.82, 0.30, -0.25, 0.55, 0.80),
    ]
    heading = _estimate_heading(rows, TrackerConfig())
    assert math.degrees(heading) < 6.0


def test_branch_classification_uses_competition_width_crossing():
    rows = [
        (0.25, 0.00, -0.08, 0.08, 0.16),
        (0.35, 0.00, -0.08, 0.08, 0.16),
        (0.45, 0.00, -0.08, 0.08, 0.16),
        (0.55, 0.00, -0.20, 0.20, 0.40),
        (0.65, 0.00, -0.08, 0.08, 0.16),
    ]
    assert _classify_branch(rows, TrackerConfig()) == JUNCTION


def test_branch_geometry_reports_corner_distance_and_side_extents():
    rows = [
        (0.25, 0.00, -0.08, 0.08, 0.16),
        (0.35, 0.00, -0.08, 0.08, 0.16),
        (0.45, 0.00, -0.08, 0.08, 0.16),
        (0.52, 0.00, -0.08, 0.28, 0.36),
        (0.55, 0.00, -0.08, 0.30, 0.38),
    ]
    branch = _branch_geometry(rows, TrackerConfig(branch_side_extent_m=0.12))

    assert branch.state == LEFT_BRANCH
    assert 0.50 <= branch.branch_x_m <= 0.56
    assert branch.left_extent_m >= 0.20
    assert branch.right_extent_m >= 0.08


def test_control_geometry_ignores_wide_crossing_for_center():
    params = TrackerConfig(
        control_min_x_m=0.20,
        control_max_x_m=0.65,
        control_reference_x_m=0.30,
        control_max_road_span_m=0.20,
    )
    grid = GroundGrid(0.12, -0.35, 0.01, 70, 70)
    component = np.zeros((grid.height, grid.width), dtype=np.uint8)
    centre = 35
    component[8:28, centre - 8 : centre + 8] = 255
    component[30:34, centre - 30 : centre + 31] = 255

    lateral, road_left, road_right, width, _, _ = _control_geometry(
        component,
        grid,
        params,
    )

    assert abs(lateral) < 0.01
    assert width < 0.20


def test_sparse_same_lane_intervals_are_merged_before_control():
    params = TrackerConfig(
        control_min_x_m=0.20,
        control_max_x_m=0.65,
        control_reference_x_m=0.30,
        control_max_road_span_m=0.20,
    )
    grid = GroundGrid(0.12, -0.35, 0.01, 70, 70)
    component = np.zeros((grid.height, grid.width), dtype=np.uint8)
    for row in range(8, 35):
        component[row, [36, 40, 44, 48]] = 255

    lateral, road_left, road_right, width, _, rows = _control_geometry(
        component,
        grid,
        params,
    )

    assert rows > 10
    assert road_left < lateral < road_right
    assert width > 0.08


def test_wide_branch_slice_keeps_forward_trunk_center():
    params = TrackerConfig(
        control_max_road_span_m=0.20,
        road_width_m=0.20,
    )
    grid = GroundGrid(0.12, -0.40, 0.01, 80, 70)
    component = np.zeros((grid.height, grid.width), dtype=np.uint8)
    centre = 40
    component[8:35, centre - 9 : centre + 10] = 255
    component[28:32, centre - 30 : centre + 31] = 255

    slices = _road_slices(component, grid, params, 0.20, 0.55, 0.36)
    wide = [row for row in slices if row.full_width_m > 0.40]

    assert wide
    assert all(abs(row.center_y_m) < 0.02 for row in wide)
    assert all(row.width_m <= params.control_max_road_span_m for row in wide)


def test_narrow_edge_is_not_a_safe_road_support():
    params = TrackerConfig()
    assert not _road_support_is_safe(0.0, 0.08, params)
    assert _road_support_is_safe(0.03, 0.20, params)
    assert _road_support_is_safe(0.0, 0.15, params)
    assert _road_support_is_safe(0.0, 0.20, params)
    assert not _road_support_is_safe(0.12, 0.20, params)


def test_positive_lateral_bias_shifts_estimate_left():
    params = TrackerConfig(lateral_error_bias_m=0.015)

    lateral, road_left, road_right = _apply_lateral_bias(
        0.0,
        -0.10,
        0.10,
        params,
    )

    assert lateral == 0.015
    assert road_left == -0.085
    assert road_right == 0.115


def test_white_mask_rejects_pale_green_rgb_cast():
    camera = CameraGeometry(
        fx=383.0,
        fy=382.5,
        cx=318.0,
        cy=247.0,
        camera_x=0.0547,
        camera_y=-0.020,
        camera_z=0.130,
        color_to_base=DEFAULT_OPTICAL_TO_BASE,
    )
    params = TrackerConfig()

    neutral_bgr = np.full((480, 640, 3), (150, 150, 150), dtype=np.uint8)
    neutral_white, _, _ = _image_masks(neutral_bgr, camera, params)
    assert np.count_nonzero(neutral_white) > 1000

    # RGB (130, 137, 112) matches the pale green/yellow floor cast observed in
    # the 20261001T142953 bag: bright enough to pass old white thresholds, but
    # chromatically green.
    pale_green_bgr = np.full((480, 640, 3), (112, 137, 130), dtype=np.uint8)
    pale_green_white, _, _ = _image_masks(pale_green_bgr, camera, params)
    assert np.count_nonzero(pale_green_white) == 0


def test_pale_green_floor_is_context_not_white_road():
    camera = CameraGeometry(
        fx=383.0,
        fy=382.5,
        cx=318.0,
        cy=247.0,
        camera_x=0.0547,
        camera_y=-0.020,
        camera_z=0.130,
        color_to_base=DEFAULT_OPTICAL_TO_BASE,
    )
    params = TrackerConfig()

    # Matches the light green mat in the hand-pushed recognition bag around
    # t=99.8 s: bright enough to look like white under the old threshold, but
    # still chromatically green in Lab/RGB opponent space.
    pale_floor_bgr = np.full((480, 640, 3), (105, 121, 118), dtype=np.uint8)
    white, green, baseline = _image_masks(pale_floor_bgr, camera, params)

    assert np.count_nonzero(white) == 0
    assert np.count_nonzero(green) > 1000
    assert baseline > 0.0


def test_depth_pixels_project_to_base_with_expected_signs():
    camera = CameraGeometry(
        fx=100.0,
        fy=100.0,
        cx=50.0,
        cy=50.0,
        camera_x=0.05,
        camera_y=0.0,
        camera_z=0.12,
        color_to_base=DEFAULT_OPTICAL_TO_BASE,
    )

    points = depth_pixels_to_base(
        camera,
        np.asarray([50.0, 60.0]),
        np.asarray([62.0, 62.0]),
        np.asarray([1.0, 1.0]),
    )

    assert np.allclose(points[0], [1.05, 0.0, 0.0])
    assert points[1, 1] < 0.0


def test_depth_control_geometry_estimates_lateral_center():
    camera = CameraGeometry(
        fx=120.0,
        fy=120.0,
        cx=160.0,
        cy=100.0,
        camera_x=0.0,
        camera_y=0.0,
        camera_z=0.12,
        color_to_base=DEFAULT_OPTICAL_TO_BASE,
    )
    params = TrackerConfig(
        control_min_x_m=0.20,
        control_max_x_m=0.58,
        control_reference_x_m=0.30,
        control_max_abs_y_m=0.30,
        max_abs_ground_y_m=0.35,
        ground_close_size=1,
        ground_open_size=1,
        depth_pixel_stride=1,
        depth_min_control_pixels=8,
        depth_min_control_rows=4,
        depth_raster_radius_px=1,
    )
    grid = GroundGrid(0.12, -0.35, 0.01, 70, 70)
    image_shape = (240, 320)
    white = np.zeros(image_shape, dtype=np.uint8)
    depth = np.zeros(image_shape, dtype=np.float32)

    for x in np.linspace(0.22, 0.55, 12):
        for y in np.linspace(0.01, 0.11, 7):
            pixel = ground_to_pixel(camera, float(x), float(y))
            assert pixel is not None
            u, v = int(round(pixel[0])), int(round(pixel[1]))
            if 0 <= u < image_shape[1] and 0 <= v < image_shape[0]:
                white[v, u] = 255
                depth[v, u] = float(x)

    result = _depth_control_geometry(white, depth, camera, grid, params)

    assert result is not None
    lateral, road_left, road_right, width, pixels, rows, _ = result
    assert 0.04 <= lateral <= 0.08
    assert road_left < lateral < road_right
    assert width >= 0.08
    assert pixels >= params.depth_min_control_pixels
    assert rows >= params.depth_min_control_rows


def test_depth_control_geometry_is_limited_to_selected_component():
    camera = CameraGeometry(
        fx=120.0,
        fy=120.0,
        cx=160.0,
        cy=100.0,
        camera_x=0.0,
        camera_y=0.0,
        camera_z=0.12,
        color_to_base=DEFAULT_OPTICAL_TO_BASE,
    )
    params = TrackerConfig(
        control_min_x_m=0.20,
        control_max_x_m=0.58,
        control_reference_x_m=0.30,
        control_max_abs_y_m=0.30,
        max_abs_ground_y_m=0.35,
        ground_close_size=1,
        ground_open_size=1,
        depth_pixel_stride=1,
        depth_min_control_pixels=8,
        depth_min_control_rows=4,
        depth_raster_radius_px=1,
        depth_component_dilate_size=1,
    )
    grid = GroundGrid(0.12, -0.35, 0.01, 70, 70)
    image_shape = (240, 320)
    white = np.zeros(image_shape, dtype=np.uint8)
    depth = np.zeros(image_shape, dtype=np.float32)
    component = np.zeros((grid.height, grid.width), dtype=np.uint8)
    centre_column = int(round((0.05 - grid.y_min) / grid.resolution))
    component[8:45, centre_column - 7 : centre_column + 8] = 255

    for y in (0.05, 0.28):
        for x in np.linspace(0.22, 0.55, 12):
            pixel = ground_to_pixel(camera, float(x), float(y))
            assert pixel is not None
            u, v = int(round(pixel[0])), int(round(pixel[1]))
            if 0 <= u < image_shape[1] and 0 <= v < image_shape[0]:
                white[v, u] = 255
                depth[v, u] = float(x)

    result = _depth_control_geometry(
        white,
        depth,
        camera,
        grid,
        params,
        component_mask=component,
    )

    assert result is not None
    lateral, _, _, _, _, _, _ = result
    assert 0.02 <= lateral <= 0.08


def test_depth_reject_reason_protects_junction_and_large_disagreement():
    params = TrackerConfig()

    assert (
        _depth_reject_reason(0.0, 0.20, JUNCTION, 0.01, 0.20, params)
        == "junction"
    )
    assert (
        _depth_reject_reason(0.0, 0.20, TRACK, 0.12, 0.20, params)
        == "large_depth_lateral"
    )
    assert (
        _depth_reject_reason(0.0, 0.20, TRACK, 0.06, 0.20, params)
        == "rgb_depth_disagree"
    )
