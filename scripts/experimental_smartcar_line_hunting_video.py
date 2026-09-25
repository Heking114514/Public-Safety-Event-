#!/usr/bin/env python3
"""Offline video test for the SmartCarLineHunting image algorithm.

This is intentionally isolated from the ROS runtime nodes. It reproduces the
repository's grayscale/Otsu, fixed-height scan, left/right edge and centerline
path, then writes a normal MJPG AVI for inspection on Ubuntu.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

import offline_image_track_correction as bag_utils


DEFAULT_BAG = Path(
    "/media/hjh/E1/rosbag_recording/navigation_runs/"
    "20260925T061444.606293Z_d711dfc7c037/bag"
)
DEFAULT_OUT = Path(
    "/media/hjh/E1/rosbag_recording/analysis/"
    "20260925T061444.606293Z_d711dfc7c037/"
    "smartcar_line_hunting.avi"
)


def image_to_bgr(message: Image) -> np.ndarray | None:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding in ("rgb8", "bgr8"):
        channels = 3
        row_width = message.step // channels
        image = data.reshape((message.height, row_width, channels))[
            :, : message.width
        ]
        if message.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()
    if message.encoding in ("mono8", "8UC1"):
        image = data.reshape((message.height, message.step))[:, : message.width]
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if message.encoding in ("rgba8", "bgra8"):
        channels = 4
        row_width = message.step // channels
        image = data.reshape((message.height, row_width, channels))[
            :, : message.width
        ]
        code = cv2.COLOR_RGBA2BGR if message.encoding == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(image, code)
    return None


def nearest_white_run(binary: np.ndarray, x: int, high: int) -> int:
    count = 0
    y = high - 1
    while y >= 0 and binary[y, x] != 0:
        count += 1
        y -= 1
    return count


def append_continuous(points: list[tuple[int, int]], point: tuple[int, int]) -> None:
    if not points:
        points.append(point)
        return
    if (
        abs(point[0] - points[-1][0]) <= 3
        and abs(point[1] - points[-1][1]) <= 3
    ):
        points.append(point)


def line_hunting(binary: np.ndarray, fraction: float = 0.73) -> dict:
    """Reproduce the repository's fixed-height white-region scan.

    The upstream code downsamples to 94x60 for inverse perspective display but
    performs the edge scan on the thresholded image. This function follows the
    scan on the thresholded image and protects the original's border accesses.
    """

    height, width = binary.shape[:2]
    high = max(1, min(height, int(height * fraction)))
    runs = np.array(
        [nearest_white_run(binary, x, high) for x in range(width)],
        dtype=np.int32,
    )
    if int(runs.max(initial=0)) <= 0:
        return {
            "left": [],
            "right": [],
            "center": [],
            "left_anchor": 0,
            "right_anchor": width - 1,
            "left_run": 0,
            "right_run": 0,
            "raw_cross": False,
        }

    left_anchor = int(np.argmax(runs))
    right_anchor = int(width - 1 - np.argmax(runs[::-1]))
    left_run = int(runs[left_anchor])
    right_run = int(runs[right_anchor])

    left_points: list[tuple[int, int]] = []
    right_points: list[tuple[int, int]] = []
    left_open_count = 0
    right_open_count = 0

    start_y = high - 1
    stop_y = max(0, high - max(left_run, right_run))
    for y in range(start_y, stop_y - 1, -1):
        left_edge = None
        x_start = min(max(left_anchor, 1), width - 1)
        for x in range(x_start, 0, -1):
            if binary[y, x] != 0 and binary[y, x - 1] == 0:
                left_edge = x
                break
            if x >= 2 and binary[y, x] != 0:
                if binary[y, x - 1] != 0 and binary[y, x - 2] != 0:
                    left_open_count += 1

        right_edge = None
        x_start = min(max(right_anchor - 4, 0), max(0, width - 2))
        for x in range(x_start, width - 1):
            if binary[y, x] != 0 and binary[y, x + 1] == 0:
                right_edge = x
                break
            if x + 2 < width and binary[y, x] != 0:
                if binary[y, x + 1] != 0 and binary[y, x + 2] != 0:
                    right_open_count += 1

        if left_edge is not None:
            append_continuous(left_points, (left_edge, y))
        if right_edge is not None:
            append_continuous(right_points, (right_edge, y))

    raw_cross = (
        abs(left_open_count - left_anchor) <= 5
        and abs(width - right_anchor + 4 - right_open_count) <= 5
    )
    center_points = [
        (
            (left_points[index][0] + right_points[index][0]) // 2,
            (left_points[index][1] + right_points[index][1]) // 2,
        )
        for index in range(min(len(left_points), len(right_points)))
    ]
    return {
        "left": left_points,
        "right": right_points,
        "center": center_points,
        "left_anchor": left_anchor,
        "right_anchor": right_anchor,
        "left_run": left_run,
        "right_run": right_run,
        "raw_cross": raw_cross,
    }


def draw_points(
    panel: np.ndarray,
    points: list[tuple[int, int]],
    scale_x: float,
    scale_y: float,
    color: tuple[int, int, int],
    radius: int = 2,
) -> None:
    for x, y in points:
        cv2.circle(
            panel,
            (int(round(x * scale_x)), int(round(y * scale_y))),
            radius,
            color,
            -1,
            cv2.LINE_AA,
        )


def render_frame(
    bgr: np.ndarray,
    binary: np.ndarray,
    result: dict,
    frame_index: int,
    stamp: float,
    cross_count: int,
    threshold: float,
    output_width: int,
    output_height: int,
) -> np.ndarray:
    panel = bgr.copy()
    height, width = binary.shape
    roi_y = int(round(height * 0.73))
    cv2.line(panel, (0, roi_y), (width - 1, roi_y), (255, 255, 0), 2)

    sx = width / 94.0
    sy = height / 60.0
    draw_points(panel, result["left"], sx, sy, (255, 80, 80), 2)
    draw_points(panel, result["right"], sx, sy, (80, 80, 255), 2)
    draw_points(panel, result["center"], sx, sy, (80, 230, 80), 2)

    if result["left"]:
        cv2.circle(
            panel,
            (
                int(round(result["left_anchor"] * sx)),
                int(round((height * 0.73 - 1) * sy)),
            ),
            5,
            (255, 180, 0),
            2,
        )
    if result["right"]:
        cv2.circle(
            panel,
            (
                int(round(result["right_anchor"] * sx)),
                int(round((height * 0.73 - 1) * sy)),
            ),
            5,
            (255, 180, 0),
            2,
        )

    stable_cross = cross_count >= 3
    label = "CROSS" if stable_cross else "LINE"
    if not result["left"] or not result["right"]:
        label = "NO BOTH EDGES"
    cv2.putText(
        panel,
        label,
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.85,
        (0, 0, 255) if stable_cross else (0, 210, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        (
            f"frame={frame_index} t={stamp:.2f}s "
            f"otsu={threshold:.0f} left={len(result['left'])} "
            f"right={len(result['right'])} center={len(result['center'])}"
        ),
        (16, 62),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    binary_panel = cv2.resize(
        binary_bgr,
        (width, height),
        interpolation=cv2.INTER_NEAREST,
    )
    draw_points(binary_panel, result["left"], 1.0, 1.0, (255, 80, 80), 2)
    draw_points(binary_panel, result["right"], 1.0, 1.0, (80, 80, 255), 2)
    draw_points(binary_panel, result["center"], 1.0, 1.0, (80, 230, 80), 2)
    cv2.line(binary_panel, (0, roi_y), (width - 1, roi_y), (255, 255, 0), 2)
    cv2.putText(
        binary_panel,
        "Otsu binary / hunted edges",
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 200, 255),
        2,
        cv2.LINE_AA,
    )

    left = cv2.resize(panel, (output_width // 2, output_height))
    right = cv2.resize(binary_panel, (output_width // 2, output_height))
    return np.hstack((left, right))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--image-topic",
        default="/camera/camera/color/image_raw",
    )
    parser.add_argument("--image-stride", type=int, default=3)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--roi-fraction", type=float, default=0.73)
    parser.add_argument("--invert", action="store_true")
    parser.add_argument("--csv", type=Path, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.image_stride < 1:
        raise ValueError("--image-stride must be positive")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    cache_dir = args.out.parent / "decompressed"
    db_paths = bag_utils.decompress_bag_files(args.bag, cache_dir)
    csv_path = args.csv or args.out.with_suffix(".csv")

    writer = None
    csv_stream = None
    csv_writer = None
    processed = 0
    written = 0
    cross_count = 0
    total_frames = 0
    edge_frames = 0
    cross_frames = 0

    try:
        csv_stream = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.DictWriter(
            csv_stream,
            fieldnames=[
                "frame",
                "stamp",
                "threshold",
                "white_ratio",
                "left_points",
                "right_points",
                "center_points",
                "raw_cross",
                "stable_cross",
            ],
        )
        csv_writer.writeheader()

        for timestamp_ns, data in bag_utils.iter_topic(db_paths, args.image_topic):
            total_frames += 1
            if processed % args.image_stride != 0:
                processed += 1
                continue
            if args.max_frames and written >= args.max_frames:
                break
            processed += 1
            message = deserialize_message(data, Image)
            bgr = image_to_bgr(message)
            if bgr is None:
                continue
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            threshold, binary = cv2.threshold(
                blurred,
                0,
                255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU,
            )
            if args.invert:
                binary = cv2.bitwise_not(binary)

            result = line_hunting(binary, args.roi_fraction)
            if result["raw_cross"]:
                cross_count += 1
                cross_frames += 1
            else:
                cross_count = max(0, cross_count - 1)
            if result["left"] and result["right"]:
                edge_frames += 1

            stamp = bag_utils.stamp_seconds(message, timestamp_ns)
            frame = render_frame(
                bgr,
                binary,
                result,
                written,
                stamp,
                cross_count,
                threshold,
                1280,
                360,
            )
            if writer is None:
                writer = cv2.VideoWriter(
                    str(args.out),
                    cv2.VideoWriter_fourcc(*"MJPG"),
                    args.fps,
                    (frame.shape[1], frame.shape[0]),
                )
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open {args.out}")
            writer.write(frame)
            csv_writer.writerow(
                {
                    "frame": written,
                    "stamp": f"{stamp:.9f}",
                    "threshold": f"{threshold:.3f}",
                    "white_ratio": f"{float(np.mean(binary > 0)):.6f}",
                    "left_points": len(result["left"]),
                    "right_points": len(result["right"]),
                    "center_points": len(result["center"]),
                    "raw_cross": int(result["raw_cross"]),
                    "stable_cross": int(cross_count >= 3),
                }
            )
            written += 1
            if written % 250 == 0:
                print(
                    f"processed={processed} written={written} "
                    f"edges={edge_frames} raw_cross={cross_frames}",
                    flush=True,
                )
    finally:
        if writer is not None:
            writer.release()
        if csv_stream is not None:
            csv_stream.close()

    print(f"bag_frames_seen={total_frames}")
    print(f"frames_written={written}")
    print(f"frames_with_both_edges={edge_frames}")
    print(f"raw_cross_frames={cross_frames}")
    print(f"video={args.out}")
    print(f"csv={csv_path}")


if __name__ == "__main__":
    main()
