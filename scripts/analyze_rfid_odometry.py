#!/usr/bin/env python3
"""Compare fused odometry against fixed RFID task-point coordinates."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import offline_image_track_correction as correction


RFID_CARD_PRESENT = 0x02
RFID_NUMBER_VALID = 0x20


def arena_to_map(point: tuple[float, float], origin: tuple[float, float], yaw: float):
    dx = point[0] - origin[0]
    dy = point[1] - origin[1]
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def load_reference(map_path: Path) -> dict[int, tuple[float, float]]:
    config = yaml.safe_load(map_path.read_text(encoding="utf-8"))
    nodes = config["nodes"]
    origin = tuple(map(float, config["frame"]["map_origin_m"]))
    yaw = math.radians(float(config["frame"]["map_x_heading_deg"]))
    return {
        int(number): arena_to_map(tuple(map(float, nodes[node])), origin, yaw)
        for number, node in config["tasks"].items()
    }


def load_odometry(db_paths: list[Path], topic: str):
    samples = []
    for timestamp_ns, data in correction.iter_topic(db_paths, topic):
        message = deserialize_message(data, Odometry)
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        samples.append(
            (
                int(timestamp_ns),
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                yaw,
            )
        )
    if not samples:
        raise RuntimeError(f"{topic} has no messages")
    return np.asarray(samples, dtype=float)


def parse_rfid(line: str):
    fields = line.split(",")
    if len(fields) != 9 or fields[0] != "RFID":
        return None
    try:
        sequence = int(fields[2])
        status = int(fields[3])
        number = int(fields[7])
    except ValueError:
        return None
    if (
        status & (RFID_CARD_PRESENT | RFID_NUMBER_VALID)
        != RFID_CARD_PRESENT | RFID_NUMBER_VALID
        or number not in range(1, 13)
    ):
        return None
    return sequence, number, fields[8]


def load_events(db_paths: list[Path]):
    events = []
    previous = None
    for timestamp_ns, data in correction.iter_topic(db_paths, "/cup_car_serial/rx"):
        parsed = parse_rfid(deserialize_message(data, String).data)
        if parsed is None or parsed == previous:
            continue
        previous = parsed
        events.append((int(timestamp_ns), *parsed))
    return events


def interpolate_pose(samples: np.ndarray, timestamp_ns: int):
    times = samples[:, 0]
    upper = int(np.searchsorted(times, timestamp_ns, side="left"))
    if upper <= 0:
        return samples[0, 1:4]
    if upper >= len(samples):
        return samples[-1, 1:4]
    before = samples[upper - 1]
    after = samples[upper]
    fraction = (timestamp_ns - before[0]) / (after[0] - before[0])
    yaw_delta = math.atan2(math.sin(after[3] - before[3]), math.cos(after[3] - before[3]))
    return np.asarray(
        [
            before[1] + fraction * (after[1] - before[1]),
            before[2] + fraction * (after[2] - before[2]),
            math.atan2(
                math.sin(before[3] + fraction * yaw_delta),
                math.cos(before[3] + fraction * yaw_delta),
            ),
        ]
    )


def rigid_alignment(observed: np.ndarray, expected: np.ndarray):
    observed_center = observed.mean(axis=0)
    expected_center = expected.mean(axis=0)
    covariance = (observed - observed_center).T @ (expected - expected_center)
    left, _singular, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right[-1, :] *= -1.0
        rotation = right.T @ left.T
    translation = expected_center - rotation @ observed_center
    return rotation, translation


def similarity_alignment(
    observed: np.ndarray, expected: np.ndarray, allow_reflection: bool
):
    observed_center = observed.mean(axis=0)
    expected_center = expected.mean(axis=0)
    observed_centered = observed - observed_center
    expected_centered = expected - expected_center
    left, singular, right = np.linalg.svd(observed_centered.T @ expected_centered)
    rotation = right.T @ left.T
    if not allow_reflection and np.linalg.det(rotation) < 0.0:
        right[-1, :] *= -1.0
        singular[-1] *= -1.0
        rotation = right.T @ left.T
    scale = float(singular.sum() / np.sum(observed_centered ** 2))
    translation = expected_center - scale * rotation @ observed_center
    aligned = (scale * rotation @ observed.T).T + translation
    errors = np.linalg.norm(aligned - expected, axis=1)
    return {
        "scale_observed_to_reference": scale,
        "observed_scale_relative_to_reference": 1.0 / scale,
        "yaw_rad": float(math.atan2(rotation[1, 0], rotation[0, 0])),
        "reflection": bool(np.linalg.det(rotation) < 0.0),
        "translation_x_m": float(translation[0]),
        "translation_y_m": float(translation[1]),
        "residual_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "residual_max_m": float(errors.max()),
    }


def analyze(bag: Path, map_path: Path, output: Path, odom_topic: str) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    db_paths = sorted(bag.glob("*.db3")) if bag.is_dir() else [bag]
    if not db_paths:
        raise RuntimeError(f"no decompressed .db3 files under {bag}")
    reference = load_reference(map_path)
    odometry = load_odometry(db_paths, odom_topic)
    events = load_events(db_paths)
    if not events:
        raise RuntimeError("no valid RFID events found")

    rows = []
    for event_index, (timestamp_ns, sequence, number, uid) in enumerate(events, 1):
        x, y, yaw = interpolate_pose(odometry, timestamp_ns)
        expected_x, expected_y = reference[number]
        error_x = x - expected_x
        error_y = y - expected_y
        rows.append(
            {
                "event": event_index,
                "timestamp_ns": timestamp_ns,
                "sequence": sequence,
                "number": number,
                "uid": uid,
                "x_m": float(x),
                "y_m": float(y),
                "yaw_rad": float(yaw),
                "expected_x_m": expected_x,
                "expected_y_m": expected_y,
                "error_x_m": float(error_x),
                "error_y_m": float(error_y),
                "error_m": float(math.hypot(error_x, error_y)),
            }
        )

    with (output / "rfid_events.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    point_rows = []
    for number in sorted({row["number"] for row in rows}):
        selected = [row for row in rows if row["number"] == number]
        observed = np.asarray([[row["x_m"], row["y_m"]] for row in selected])
        mean = observed.mean(axis=0)
        expected = np.asarray(reference[number])
        spread = np.sqrt(np.mean(np.sum((observed - mean) ** 2, axis=1)))
        point_rows.append(
            {
                "number": number,
                "events": len(selected),
                "mean_x_m": float(mean[0]),
                "mean_y_m": float(mean[1]),
                "expected_x_m": float(expected[0]),
                "expected_y_m": float(expected[1]),
                "error_x_m": float(mean[0] - expected[0]),
                "error_y_m": float(mean[1] - expected[1]),
                "error_m": float(np.linalg.norm(mean - expected)),
                "repeat_spread_rms_m": float(spread),
            }
        )
    with (output / "rfid_points.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=point_rows[0].keys())
        writer.writeheader()
        writer.writerows(point_rows)

    observed = np.asarray([[row["mean_x_m"], row["mean_y_m"]] for row in point_rows])
    expected = np.asarray([[row["expected_x_m"], row["expected_y_m"]] for row in point_rows])
    raw_errors = np.linalg.norm(observed - expected, axis=1)
    summary = {
        "bag": str(bag),
        "map": str(map_path),
        "odometry_topic": odom_topic,
        "assumption": "RFID tag coordinate equals base_link coordinate at detection",
        "events": len(rows),
        "unique_points": len(point_rows),
        "raw_rmse_m": float(np.sqrt(np.mean(raw_errors ** 2))),
        "raw_mean_error_m": float(raw_errors.mean()),
        "raw_max_error_m": float(raw_errors.max()),
        "mean_error_x_m": float(np.mean(observed[:, 0] - expected[:, 0])),
        "mean_error_y_m": float(np.mean(observed[:, 1] - expected[:, 1])),
        "mean_repeat_spread_rms_m": float(
            np.mean([row["repeat_spread_rms_m"] for row in point_rows])
        ),
        "points": point_rows,
    }
    if len(point_rows) >= 2:
        rotation, translation = rigid_alignment(observed, expected)
        aligned = (rotation @ observed.T).T + translation
        aligned_errors = np.linalg.norm(aligned - expected, axis=1)
        summary["best_fit_rigid"] = {
            "yaw_rad": float(math.atan2(rotation[1, 0], rotation[0, 0])),
            "translation_x_m": float(translation[0]),
            "translation_y_m": float(translation[1]),
            "residual_rmse_m": float(np.sqrt(np.mean(aligned_errors ** 2))),
            "residual_max_m": float(aligned_errors.max()),
        }
        summary["best_fit_similarity"] = similarity_alignment(
            observed, expected, allow_reflection=False
        )
        summary["best_fit_similarity_with_reflection"] = similarity_alignment(
            observed, expected, allow_reflection=True
        )

    (output / "rfid_odometry_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    axes[0].scatter(expected[:, 0], expected[:, 1], marker="x", label="RFID reference")
    axes[0].scatter(observed[:, 0], observed[:, 1], marker="o", label="fused odometry")
    for point, measured, row in zip(expected, observed, point_rows):
        axes[0].plot([point[0], measured[0]], [point[1], measured[1]], color="gray")
        axes[0].annotate(str(row["number"]), measured)
    axes[0].set_aspect("equal")
    axes[0].set_xlabel("map x (m)")
    axes[0].set_ylabel("map y (m)")
    axes[0].grid(True)
    axes[0].legend()
    axes[0].set_title("RFID reference vs fused odometry")

    axes[1].bar([str(row["number"]) for row in point_rows], raw_errors)
    axes[1].set_xlabel("RFID number")
    axes[1].set_ylabel("position error (m)")
    axes[1].grid(True, axis="y")
    axes[1].set_title("Mean position error at each RFID point")
    figure.tight_layout()
    figure.savefig(output / "rfid_odometry_error.png", dpi=160)
    plt.close(figure)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--odom-topic", default="/odometry/fused")
    args = parser.parse_args()
    try:
        summary = analyze(
            args.bag.resolve(), args.map.resolve(), args.output_dir.resolve(),
            args.odom_topic,
        )
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print(f"RFID odometry analysis failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
