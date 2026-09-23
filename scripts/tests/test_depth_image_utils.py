"""Contract tests for the dependency-free depth image helpers.

The utility is intentionally tested through a small sensor_msgs/Image-shaped
object so these tests do not require ROS or cv_bridge.  The expected API is:

    decode_depth_image(image, depth_scale=...) -> numpy.ndarray
    sample_valid_depth(
        depth_m, u, v, radius_px=..., min_depth_m=..., max_depth_m=...
    ) -> float | None

Decoded values are expressed in metres.  ``16UC1`` defaults to millimetres
and ``32FC1`` defaults to metres unless an explicit scale is supplied.
"""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import struct
import sys
import unittest
from types import SimpleNamespace
from typing import Any

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))


try:
    _utils = importlib.import_module("depth_image_utils")
except ModuleNotFoundError:
    _utils = None


def _image(
    *,
    width: int,
    height: int,
    encoding: str,
    values: bytes,
    is_bigendian: bool = False,
    step: int | None = None,
) -> Any:
    bytes_per_pixel = 2 if encoding == "16UC1" else 4
    return SimpleNamespace(
        width=width,
        height=height,
        encoding=encoding,
        is_bigendian=int(is_bigendian),
        step=step if step is not None else width * bytes_per_pixel,
        data=values,
    )


def _uint16_bytes(rows: list[list[int]], *, bigendian: bool = False) -> bytes:
    prefix = ">" if bigendian else "<"
    return b"".join(struct.pack(prefix + "H", value) for row in rows for value in row)


def _float32_bytes(rows: list[list[float]], *, bigendian: bool = False) -> bytes:
    prefix = ">" if bigendian else "<"
    return b"".join(struct.pack(prefix + "f", value) for row in rows for value in row)


@unittest.skipUnless(
    _utils is not None,
    "scripts/depth_image_utils.py is not present yet; this is the integration contract",
)
class DepthImageUtilsTest(unittest.TestCase):
    def test_decodes_little_endian_16uc1_to_metres(self) -> None:
        image = _image(
            width=2,
            height=2,
            encoding="16UC1",
            values=_uint16_bytes([[500, 1250], [0, 3000]]),
        )

        decoded = _utils.decode_depth_image(image)

        np.testing.assert_allclose(decoded, [[0.5, 1.25], [0.0, 3.0]])

    def test_decodes_big_endian_16uc1_and_honours_row_step(self) -> None:
        rows = [[500, 1250], [750, 3000]]
        row_bytes = [_uint16_bytes([row], bigendian=True) for row in rows]
        padding = b"\xA5\x5A"
        payload = b"".join(row + padding for row in row_bytes)
        image = _image(
            width=2,
            height=2,
            encoding="16UC1",
            values=payload,
            is_bigendian=True,
            step=6,
        )

        decoded = _utils.decode_depth_image(image)

        np.testing.assert_allclose(decoded, [[0.5, 1.25], [0.75, 3.0]])

    def test_decodes_little_endian_32fc1_in_metres(self) -> None:
        image = _image(
            width=2,
            height=1,
            encoding="32FC1",
            values=_float32_bytes([[0.25, 1.75]]),
        )

        decoded = _utils.decode_depth_image(image)

        self.assertEqual(decoded[0][0], 0.25)
        self.assertEqual(decoded[0][1], 1.75)

    def test_decodes_big_endian_32fc1(self) -> None:
        image = _image(
            width=2,
            height=1,
            encoding="32FC1",
            values=_float32_bytes([[0.375, 2.5]], bigendian=True),
            is_bigendian=True,
        )

        decoded = _utils.decode_depth_image(image)

        self.assertTrue(math.isclose(decoded[0][0], 0.375, rel_tol=0.0, abs_tol=1e-7))
        self.assertTrue(math.isclose(decoded[0][1], 2.5, rel_tol=0.0, abs_tol=1e-7))

    def test_neighborhood_median_ignores_invalid_depth_values(self) -> None:
        image = _image(
            width=3,
            height=3,
            encoding="32FC1",
            values=_float32_bytes(
                [
                    [0.0, 1.0, math.nan],
                    [math.inf, 2.0, -1.0],
                    [0.5, 100.0, 3.0],
                ]
            ),
        )
        decoded = _utils.decode_depth_image(image)

        median = _utils.sample_valid_depth(
            decoded,
            u=1,
            v=1,
            radius_px=1,
            min_depth_m=0.1,
            max_depth_m=10.0,
        )

        # Valid samples are 1.0, 2.0, 0.5 and 3.0; the median is 1.5.
        self.assertEqual(median, 1.5)

    def test_neighborhood_median_returns_none_when_all_values_are_invalid(self) -> None:
        image = _image(
            width=2,
            height=2,
            encoding="16UC1",
            values=_uint16_bytes([[0, 0], [0, 0]]),
        )
        decoded = _utils.decode_depth_image(image)

        median = _utils.sample_valid_depth(
            decoded,
            u=0,
            v=0,
            radius_px=1,
            min_depth_m=0.05,
            max_depth_m=10.0,
        )

        self.assertIsNone(median)


if __name__ == "__main__":
    unittest.main()
