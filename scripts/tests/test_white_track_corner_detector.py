import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src" / "vision_correction"))

from vision_correction import road_detector
from white_track_corner_detector import (
    CROSS_JUNCTION,
    LEFT_CORNER,
    LOST,
    RIGHT_CORNER,
    STRAIGHT,
    T_JUNCTION,
    WhiteTrackParameters,
    classify_ground_mask,
)


class WhiteTrackCornerDetectorTest(unittest.TestCase):
    def setUp(self):
        self.params = WhiteTrackParameters()
        self.grid = road_detector.GroundGrid(
            self.params.min_ground_x_m,
            -self.params.max_abs_ground_y_m,
            self.params.ground_grid_resolution_m,
            150,
            125,
        )
        self.center = 75
        self.lane = slice(self.center - 10, self.center + 11)
        self.branch_row = 45

    def mask_for(self, kind):
        mask = np.zeros((self.grid.height, self.grid.width), dtype=np.uint8)
        mask[self.branch_row + 4 :, self.lane] = 255
        if kind in {"straight", "cross", "left", "right"}:
            mask[: self.branch_row + 5, self.lane] = 255
        if kind == "cross":
            mask[self.branch_row - 4 : self.branch_row + 5, 15:136] = 255
        if kind == "t":
            mask[self.branch_row - 4 : self.branch_row + 5, 15:136] = 255
            mask[: self.branch_row - 4, self.lane] = 0
        if kind == "left":
            mask[self.branch_row - 4 : self.branch_row + 5, self.center + 10 : 136] = 255
        if kind == "right":
            mask[self.branch_row - 4 : self.branch_row + 5, 15 : self.center - 10] = 255
        return mask

    def classify(self, kind):
        mask = self.mask_for(kind)
        green = np.ones_like(mask) * 255
        green[mask != 0] = 0
        return classify_ground_mask(
            mask,
            green,
            self.grid,
            self.params,
            green_fraction=0.5,
        )

    def test_straight(self):
        self.assertEqual(STRAIGHT, self.classify("straight").state)

    def test_left_and_right_corners(self):
        self.assertEqual(LEFT_CORNER, self.classify("left").state)
        self.assertEqual(RIGHT_CORNER, self.classify("right").state)

    def test_t_and_cross_junctions(self):
        self.assertEqual(T_JUNCTION, self.classify("t").state)
        self.assertEqual(CROSS_JUNCTION, self.classify("cross").state)

    def test_missing_green_context_is_lost(self):
        mask = self.mask_for("straight")
        no_green = np.zeros_like(mask)
        result = classify_ground_mask(
            mask,
            no_green,
            self.grid,
            self.params,
            green_fraction=0.0,
        )
        self.assertEqual(LOST, result.state)

    def test_side_road_is_not_current_track(self):
        mask = self.mask_for("straight")
        shifted = np.zeros_like(mask)
        shifted[:, 100:] = mask[:, :50]
        green = np.ones_like(mask) * 255
        green[shifted != 0] = 0
        result = classify_ground_mask(
            shifted,
            green,
            self.grid,
            self.params,
            green_fraction=0.5,
        )
        self.assertEqual(LOST, result.state)


if __name__ == "__main__":
    unittest.main()
