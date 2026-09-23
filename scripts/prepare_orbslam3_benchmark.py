#!/usr/bin/env python3
"""Audit SLAM replay inputs; optionally export every RGB frame for kernel tests.

Export is offline: no ROS publishers, camera, or robot node is started.
The RGB corpus is for common feature kernels, not stereo SLAM certification.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess

import yaml


RGB = "/camera/camera/color/image_raw"
LEFT = "/camera/camera/infra1/image_rect_raw"
RIGHT = "/camera/camera/infra2/image_rect_raw"


def audit(bag: Path) -> dict:
    metadata_bytes = (bag / "metadata.yaml").read_bytes()
    info = yaml.safe_load(metadata_bytes)["rosbag2_bagfile_information"]
    topics = {
        item["topic_metadata"]["name"]: {
            "type": item["topic_metadata"]["type"],
            "count": item["message_count"],
        }
        for item in info["topics_with_message_count"]
    }
    missing = [
        topic for topic in (LEFT, RIGHT)
        if topics.get(topic, {}).get("count", 0) == 0
    ]
    return {
        "bag": str(bag),
        "metadata_sha256": hashlib.sha256(metadata_bytes).hexdigest(),
        "duration_seconds": info["duration"]["nanoseconds"] / 1e9,
        "topics": topics,
        "missing_stereo_images": missing,
        "stereo_images_present": not missing,
        "stereo_certified": False,
        "note": "Topic presence alone does not verify timestamps or calibration.",
    }


def export_rgb(bag: Path, output: Path, summary: dict) -> None:
    # ROS imports are optional for metadata-only audits.
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    output.mkdir(parents=True, exist_ok=True)
    raw_path, index_path = output / "rgb.raw", output / "index.tsv"
    manifest = output / "corpus.json"
    if any(path.exists() for path in (raw_path, index_path, manifest)):
        raise ValueError("Use a new export directory to preserve existing results")
    info = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    db_paths = []
    cache = output / "decompressed"
    for relative in info["relative_file_paths"]:
        source = (bag / relative).resolve()
        if not source.is_relative_to(bag):
            raise ValueError("Shard path is outside the bag directory")
        if source.suffix == ".zstd":
            cache.mkdir(exist_ok=True)
            db_path = cache / source.name.removesuffix(".zstd")
            subprocess.run(["zstd", "-q", "-d", str(source), "-o", str(db_path)], check=True)
        elif source.suffix == ".db3":
            db_path = source
        else:
            raise ValueError(f"Unsupported storage shard: {source.name}")
        db_paths.append(db_path)

    count, shape, previous_stamp = 0, None, None
    digest = hashlib.sha256()
    with raw_path.open("xb") as raw, index_path.open("x") as index:
        for db_path in db_paths:
            with sqlite3.connect(db_path.as_uri() + "?mode=ro", uri=True) as conn:
                rows = conn.execute(
                    "SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                    "WHERE t.name=? ORDER BY m.timestamp, m.id", (RGB,),
                )
                for (data,) in rows:
                    msg = deserialize_message(data, Image)
                    if msg.encoding != "rgb8" or msg.step < msg.width * 3:
                        raise ValueError(f"Expected RGB8, got {msg.encoding}")
                    current_shape = (msg.width, msg.height)
                    if shape is not None and shape != current_shape:
                        raise ValueError("Image dimensions changed within the bag")
                    shape = current_shape
                    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                    if previous_stamp is not None and stamp_ns <= previous_stamp:
                        raise ValueError("Image timestamps are not strictly increasing")
                    previous_stamp = stamp_ns
                    pixels = bytes(msg.data)
                    if len(pixels) != msg.step * msg.height:
                        raise ValueError("Invalid image payload length")
                    for row in range(msg.height):
                        payload = pixels[row * msg.step:row * msg.step + msg.width * 3]
                        raw.write(payload)
                        digest.update(payload)
                    index.write(f"{count}\t{stamp_ns // 1_000_000_000}.{stamp_ns % 1_000_000_000:09d}\n")
                    count += 1
    expected = summary["topics"].get(RGB, {}).get("count", 0)
    if count != expected or count == 0:
        raise ValueError(f"Expected {expected} images, exported {count}")
    manifest.write_text(json.dumps({
        "source": summary,
        "frames": count,
        "width": shape[0], "height": shape[1], "encoding": "rgb8",
        "frame_stride_bytes": shape[0] * shape[1] * 3,
        "raw_sha256": digest.hexdigest(),
        "temporal_stride": 1, "spatial_scale": 1,
    }, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--export-rgb", type=Path, help="New output directory; no subsampling")
    args = parser.parse_args()
    bag = args.bag.resolve(strict=True)
    summary = audit(bag)
    print(json.dumps(summary, indent=2))
    if args.export_rgb is not None:
        export_rgb(bag, args.export_rgb.resolve(), summary)


if __name__ == "__main__":
    main()
