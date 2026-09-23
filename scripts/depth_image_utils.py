"""Small, ROS-independent helpers for decoding and sampling depth images."""

from __future__ import annotations

from numbers import Real
from typing import Any, Iterable, Optional, Tuple

import numpy as np


__all__ = [
    "decode_depth_image",
    "sample_valid_depth",
    "nearest_depth_frame",
]


def _image_encoding(message: Any) -> str:
    encoding = getattr(message, "encoding", "")
    return str(encoding).strip().lower()


def _image_shape_and_step(message: Any, itemsize: int) -> Tuple[int, int, int]:
    try:
        height = int(message.height)
        width = int(message.width)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("depth image message must provide height and width") from exc

    if height < 0 or width < 0:
        raise ValueError("depth image dimensions must be non-negative")

    step = int(getattr(message, "step", width * itemsize))
    minimum_step = width * itemsize
    if step < minimum_step:
        raise ValueError("depth image step is smaller than one row")
    return height, width, step


def decode_depth_image(message: Any, depth_scale: float = 0.001) -> np.ndarray:
    """Decode a sensor_msgs/Image-like message into float32 metres.

    Supported encodings are ``16UC1``, ``mono16``, and ``32FC1``. The
    message may use either byte order and may contain row padding in ``step``.
    """
    encoding = _image_encoding(message)
    if encoding in ("16uc1", "mono16"):
        itemsize = 2
        scale = float(depth_scale)
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("depth_scale must be a positive finite number")
        dtype_code = "u2"
    elif encoding == "32fc1":
        itemsize = 4
        scale = 1.0
        dtype_code = "f4"
    else:
        raise ValueError(
            "unsupported depth image encoding {!r}; expected 16UC1, "
            "mono16, or 32FC1".format(getattr(message, "encoding", encoding))
        )

    height, width, step = _image_shape_and_step(message, itemsize)
    data = memoryview(getattr(message, "data", b""))
    required_bytes = height * step
    if data.nbytes < required_bytes:
        raise ValueError(
            "depth image data is too short: got {} bytes, need {}".format(
                data.nbytes, required_bytes
            )
        )

    byte_order = ">" if bool(getattr(message, "is_bigendian", False)) else "<"
    dtype = np.dtype(byte_order + dtype_code)
    result = np.empty((height, width), dtype=np.float32)
    row_bytes = width * itemsize
    for row in range(height):
        start = row * step
        row_data = np.frombuffer(data[start : start + row_bytes], dtype=dtype, count=width)
        result[row, :] = row_data

    result *= np.float32(scale)
    return result


def sample_valid_depth(
    depth_m: np.ndarray,
    u: float,
    v: float,
    radius_px: int = 3,
    min_depth_m: float = 0.05,
    max_depth_m: float = 5.0,
) -> Optional[float]:
    """Return the median valid depth around pixel ``(u, v)`` in metres."""
    array = np.asarray(depth_m)
    if array.ndim != 2:
        raise ValueError("depth_m must be a two-dimensional array")

    radius = int(radius_px)
    if radius < 0:
        raise ValueError("radius_px must be non-negative")
    minimum = float(min_depth_m)
    maximum = float(max_depth_m)
    if not np.isfinite(minimum) or not np.isfinite(maximum) or minimum > maximum:
        raise ValueError("depth bounds must be finite and ordered")

    if not np.isfinite(u) or not np.isfinite(v):
        return None
    center_u = int(round(float(u)))
    center_v = int(round(float(v)))
    height, width = array.shape
    left = max(0, center_u - radius)
    right = min(width, center_u + radius + 1)
    top = max(0, center_v - radius)
    bottom = min(height, center_v + radius + 1)
    if left >= right or top >= bottom:
        return None

    neighborhood = np.asarray(array[top:bottom, left:right], dtype=np.float32)
    valid = neighborhood[
        np.isfinite(neighborhood)
        & (neighborhood >= np.float32(minimum))
        & (neighborhood <= np.float32(maximum))
    ]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def _stamp_to_seconds(stamp: Any) -> float:
    if isinstance(stamp, Real):
        return float(stamp)
    nanoseconds = getattr(stamp, "nanoseconds", None)
    if nanoseconds is not None:
        return float(nanoseconds) * 1.0e-9
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if seconds is not None and nanoseconds is not None:
        return float(seconds) + float(nanoseconds) * 1.0e-9
    if isinstance(stamp, (tuple, list)) and len(stamp) == 2:
        return float(stamp[0]) + float(stamp[1]) * 1.0e-9
    raise TypeError("unsupported timestamp type")


def nearest_depth_frame(
    frames: Iterable[Tuple[Any, Any]],
    stamp: Any,
    tolerance: float,
) -> Optional[Tuple[Any, Any]]:
    """Return the nearest ``(frame_stamp, depth_frame)`` within tolerance.

    ``frames`` contains timestamp/frame pairs. Timestamps may be seconds,
    ROS-like ``sec``/``nanosec`` objects, nanosecond objects, or ``(sec,
    nanosec)`` pairs. ``tolerance`` is measured in seconds.
    """
    max_delta = float(tolerance)
    if not np.isfinite(max_delta) or max_delta < 0.0:
        raise ValueError("tolerance must be a non-negative finite number")
    target = _stamp_to_seconds(stamp)
    nearest: Optional[Tuple[Any, Any]] = None
    nearest_delta = float("inf")

    for candidate_stamp, frame in frames:
        delta = abs(_stamp_to_seconds(candidate_stamp) - target)
        if delta < nearest_delta:
            nearest_delta = delta
            nearest = (candidate_stamp, frame)

    if nearest is None or nearest_delta > max_delta:
        return None
    return nearest
