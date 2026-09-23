#!/usr/bin/env python3
"""Extract RGB/intensity tuning frames from mono/RGB rosbag images."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

import offline_image_track_correction as correction


DEFAULT_BAG = Path("/media/hjh/Data/rosbag_recording/latest_navigation_bag")
DEFAULT_OUT = Path(
    "analysis_bags/20260921T153423.632162Z_9ead9ca47d9b/rgb_tuning_frames"
)


@dataclass(frozen=True)
class FrameSelection:
    index: int
    stamp: float
    image: np.ndarray


def parse_int_list(value: str) -> list[int]:
    result: list[int] = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(int(item))
    return result


def parse_index_set(value: str) -> set[int]:
    return set(parse_int_list(value)) if value else set()


def image_to_bgr(message: Image) -> np.ndarray | None:
    data = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding == "rgb8":
        rgb = data.reshape((message.height, message.step // 3, 3))[:, : message.width]
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if message.encoding == "bgr8":
        return data.reshape((message.height, message.step // 3, 3))[:, : message.width].copy()
    gray = correction.image_to_gray(message)
    if gray is not None:
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def roi_mask(shape: tuple[int, int], args) -> np.ndarray:
    height, width = shape
    mask = np.zeros((height, width), dtype=np.uint8)
    top = int(round(height * args.roi_top_fraction))
    bottom = int(round(height * args.roi_bottom_fraction))
    bottom = max(top + 1, min(height, bottom))
    mask[top:bottom, :] = 255
    return mask


def clean_mask(mask: np.ndarray, args) -> np.ndarray:
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (args.morphology_size, args.morphology_size)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def rgb_dark_mask(bgr: np.ndarray, threshold: int, args) -> np.ndarray:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    mask = np.all(rgb <= threshold, axis=2).astype(np.uint8) * 255
    mask = cv2.bitwise_and(mask, roi_mask(mask.shape, args))
    return clean_mask(mask, args)


def local_rgb_dark_mask(bgr: np.ndarray, delta: int, args) -> np.ndarray:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    sigma = max(1.0, args.local_sigma)
    background = cv2.GaussianBlur(gray, (0, 0), sigmaX=sigma, sigmaY=sigma)
    difference = cv2.subtract(background, gray)
    mask = cv2.threshold(difference, delta, 255, cv2.THRESH_BINARY)[1]
    mask = cv2.bitwise_and(mask, roi_mask(mask.shape, args))
    return clean_mask(mask, args)


def hough_lines(mask: np.ndarray, args):
    return cv2.HoughLinesP(
        mask,
        1.0,
        np.pi / 180.0,
        args.hough_threshold,
        minLineLength=args.min_line_length_px,
        maxLineGap=args.max_line_gap_px,
    )


def overlay_mask_and_lines(bgr: np.ndarray, mask: np.ndarray, args) -> np.ndarray:
    overlay = bgr.copy()
    red = np.zeros_like(overlay)
    red[:, :, 2] = 255
    overlay = np.where(mask[:, :, None] > 0, cv2.addWeighted(overlay, 0.45, red, 0.55, 0), overlay)
    if args.draw_hough:
        lines = hough_lines(mask, args)
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0, :]:
                cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2, cv2.LINE_AA)
    draw_roi(overlay, args)
    return overlay


def draw_roi(image: np.ndarray, args) -> None:
    height = image.shape[0]
    top = int(round(height * args.roi_top_fraction))
    bottom = int(round(height * args.roi_bottom_fraction))
    cv2.line(image, (0, top), (image.shape[1] - 1, top), (255, 255, 0), 1)
    cv2.line(image, (0, bottom), (image.shape[1] - 1, bottom), (255, 255, 0), 1)


def put_label(image: np.ndarray, text: str) -> None:
    cv2.rectangle(image, (0, 0), (image.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(
        image,
        text,
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def resize_panel(image: np.ndarray, args) -> np.ndarray:
    return cv2.resize(image, (args.panel_width, args.panel_height), interpolation=cv2.INTER_AREA)


def make_tuning_sheet(frame: FrameSelection, args) -> np.ndarray:
    panels: list[np.ndarray] = []
    raw = frame.image.copy()
    draw_roi(raw, args)
    put_label(raw, f"raw RGB view  frame={frame.index}  t={frame.stamp:.3f}")
    panels.append(resize_panel(raw, args))

    for threshold in args.rgb_thresholds[:4]:
        mask = rgb_dark_mask(frame.image, threshold, args)
        panel = overlay_mask_and_lines(frame.image, mask, args)
        put_label(panel, f"RGB <= {threshold}")
        panels.append(resize_panel(panel, args))

    for delta in args.local_deltas[:4]:
        mask = local_rgb_dark_mask(frame.image, delta, args)
        panel = overlay_mask_and_lines(frame.image, mask, args)
        put_label(panel, f"local bg - RGB >= {delta}")
        panels.append(resize_panel(panel, args))

    while len(panels) < 9:
        panels.append(np.zeros_like(panels[0]))

    rows = []
    for start in range(0, 9, 3):
        rows.append(np.hstack(panels[start : start + 3]))
    return np.vstack(rows)


def should_keep_frame(index: int, selected: set[int], args) -> bool:
    if selected:
        return index in selected
    if index < args.start_index:
        return False
    if args.end_index >= 0 and index > args.end_index:
        return False
    return (index - args.start_index) % args.stride == 0


def write_outputs(frames: list[FrameSelection], args) -> None:
    raw_dir = args.out / "raw"
    sheet_dir = args.out / "sheets"
    raw_dir.mkdir(parents=True, exist_ok=True)
    sheet_dir.mkdir(parents=True, exist_ok=True)

    index_path = args.out / "index.csv"
    with index_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame_index", "stamp", "raw_path", "sheet_path"])
        for frame in frames:
            raw_path = raw_dir / f"frame_{frame.index:04d}_raw.png"
            sheet_path = sheet_dir / f"frame_{frame.index:04d}_rgb_tuning.png"
            cv2.imwrite(str(raw_path), frame.image)
            cv2.imwrite(str(sheet_path), make_tuning_sheet(frame, args))
            writer.writerow([frame.index, f"{frame.stamp:.9f}", raw_path, sheet_path])


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--frame-indices", default="")
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--end-index", type=int, default=-1)
    parser.add_argument("--stride", type=int, default=150)
    parser.add_argument("--max-frames", type=int, default=18)
    parser.add_argument("--rgb-thresholds", type=parse_int_list, default=parse_int_list("45,60,75,90"))
    parser.add_argument("--local-deltas", type=parse_int_list, default=parse_int_list("5,9,13,17"))
    parser.add_argument("--local-sigma", type=float, default=22.0)
    parser.add_argument("--roi-top-fraction", type=float, default=0.48)
    parser.add_argument("--roi-bottom-fraction", type=float, default=0.93)
    parser.add_argument("--morphology-size", type=int, default=3)
    parser.add_argument("--hough-threshold", type=int, default=16)
    parser.add_argument("--min-line-length-px", type=int, default=14)
    parser.add_argument("--max-line-gap-px", type=int, default=10)
    parser.add_argument("--draw-hough", action="store_true")
    parser.add_argument("--panel-width", type=int, default=424)
    parser.add_argument("--panel-height", type=int, default=240)
    return parser.parse_args()


def main():
    args = parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    selected = parse_index_set(args.frame_indices)
    cache_dir = args.out / "decompressed"
    db_paths = correction.decompress_bag_files(args.bag, cache_dir)

    frames: list[FrameSelection] = []
    for index, (timestamp_ns, data) in enumerate(correction.iter_topic(db_paths, args.topic)):
        if not should_keep_frame(index, selected, args):
            continue
        message = deserialize_message(data, Image)
        bgr = image_to_bgr(message)
        if bgr is None:
            continue
        frames.append(
            FrameSelection(
                index=index,
                stamp=correction.stamp_seconds(message, timestamp_ns),
                image=bgr,
            )
        )
        if args.max_frames > 0 and len(frames) >= args.max_frames:
            break

    write_outputs(frames, args)
    print(f"wrote {len(frames)} RGB tuning frames to {args.out}")


if __name__ == "__main__":
    main()
