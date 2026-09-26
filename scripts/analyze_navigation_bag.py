#!/usr/bin/env python3
"""Analyze route segments and odometry from a recorded ROS 2 navigation bag.

The navigator publishes the *next* waypoint as current_waypoint.  Therefore,
while current_waypoint == i, the vehicle is driving from waypoint i - 1 to
waypoint i.  This script preserves that convention and reports both the
straight-line displacement and the full odometry polyline, which is important
for a chassis whose tracking point is offset from its wheel axle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOMETRY_TOPICS = (
    "/odometry/visual_raw",
    "/odometry/visual_continuous",
    "/wheel/odom",
    "/odometry/local",
    "/odometry/local_map",
    "/odometry/fused",
)
TF_TOPIC = "/tf"
PRIMARY_ODOMETRY_TOPICS = (
    "/odometry/visual_raw",
    "/odometry/visual_continuous",
    "/wheel/odom",
    "/odometry/local",
    "/odometry/local_map",
    "/odometry/fused",
)
ROUTE_TOPICS = ("/waypoint_navigation/route_input", "/waypoint_path")
COLOR_BY_TOPIC = {
    "/odometry/visual_raw": "#c2410c",
    "/odometry/visual_continuous": "#ea580c",
    "/wheel/odom": "#2563eb",
    "/odometry/local": "#16a34a",
    "/odometry/local_map": "#65a30d",
    "/odometry/fused": "#111827",
}
LABEL_BY_TOPIC = {
    "/odometry/visual_raw": "visual raw",
    "/odometry/visual_continuous": "visual continuous",
    "/wheel/odom": "wheel",
    "/odometry/local": "local EKF",
    "/odometry/local_map": "local map",
    "/odometry/fused": "fused",
}
TURN_STATES = {
    "ROTATING_TO_PATH",
    "TURN_HALF_SETTLED",
    "RECOVERING_NO_TURN_PROGRESS",
}
DRIVE_STATES = {"FOLLOWING", "RECOVERING_WAYPOINT", "REVERSING_PLANNED_RETREAT"}
BRAKE_STATES = {"BRAKING_APPROACH", "BRAKING_AT_WAYPOINT", "WAITING_AT_WAYPOINT"}
WAYPOINT_PASS_LONGITUDINAL_TOLERANCE_M = 0.01
WAYPOINT_PASS_LATERAL_TOLERANCE_M = 0.045
WAYPOINT_TOLERANCE_M = 0.04
PRE_TURN_STOP_HEADING_THRESHOLD_RAD = 0.18
DEFAULT_TURN_CENTER_OFFSET_M = 0.050


@dataclass(frozen=True)
class PoseSample:
    time_s: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float
    stop_marker: bool


@dataclass(frozen=True)
class RouteSnapshot:
    time_s: float
    waypoints: tuple[Waypoint, ...]


@dataclass(frozen=True)
class WaypointEvent:
    time_s: float
    index: int


@dataclass(frozen=True)
class StatusEvent:
    time_s: float
    state: str


@dataclass(frozen=True)
class TransformSample:
    time_s: float
    x: float
    y: float
    yaw: float


def message_time(message: Any) -> Optional[float]:
    if not hasattr(message, "header"):
        return None
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if math.isfinite(value) else None


def quaternion_yaw(quaternion: Any) -> float:
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if norm <= 1.0e-12 or not math.isfinite(norm):
        return float("nan")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def finite_or_nan(value: float) -> float:
    return float(value) if math.isfinite(float(value)) else float("nan")


def pose_from_odometry(message: Any, time_s: float) -> Optional[PoseSample]:
    yaw = quaternion_yaw(message.pose.pose.orientation)
    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        yaw,
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.angular.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return None
    return PoseSample(
        time_s=time_s,
        x=float(values[0]),
        y=float(values[1]),
        yaw=float(values[2]),
        vx=float(values[3]),
        vy=float(values[4]),
        wz=float(values[5]),
    )


def waypoint_from_pose(pose: Any) -> Waypoint:
    return Waypoint(
        x=float(pose.pose.position.x),
        y=float(pose.pose.position.y),
        yaw=quaternion_yaw(pose.pose.orientation),
        stop_marker=float(pose.pose.position.z) > 5.0e-4,
    )


def open_reader(bag_path: Path) -> tuple[Any, dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    return reader, topic_types


def collect_bag(
    bag_path: Path,
) -> tuple[
    float,
    dict[str, list[PoseSample]],
    list[TransformSample],
    list[RouteSnapshot],
    list[WaypointEvent],
    list[StatusEvent],
    dict[str, str],
]:
    reader, topic_types = open_reader(bag_path)
    selected = set(ODOMETRY_TOPICS) | set(ROUTE_TOPICS) | {
        "/waypoint_navigation/current_waypoint",
        "/waypoint_navigation/status",
        TF_TOPIC,
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in selected
        if topic in topic_types
    }
    odometry: dict[str, list[PoseSample]] = {topic: [] for topic in ODOMETRY_TOPICS}
    route_messages: list[tuple[float, Any, str]] = []
    map_odom: list[TransformSample] = []
    waypoint_events: list[WaypointEvent] = []
    status_events: list[StatusEvent] = []
    first_bag_ns: Optional[int] = None

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if first_bag_ns is None:
            first_bag_ns = int(bag_ns)
        if topic not in message_types:
            continue
        time_s = (int(bag_ns) - int(first_bag_ns)) * 1.0e-9
        message = deserialize_message(serialized, message_types[topic])
        if topic in ODOMETRY_TOPICS:
            sample = pose_from_odometry(message, time_s)
            if sample is not None:
                odometry[topic].append(sample)
        elif topic == TF_TOPIC:
            for transform in message.transforms:
                if (
                    transform.header.frame_id == "map"
                    and transform.child_frame_id == "odom"
                ):
                    map_odom.append(
                        TransformSample(
                            time_s=time_s,
                            x=float(transform.transform.translation.x),
                            y=float(transform.transform.translation.y),
                            yaw=quaternion_yaw(transform.transform.rotation),
                        )
                    )
        elif topic in ROUTE_TOPICS:
            route_messages.append((time_s, message, topic))
        elif topic == "/waypoint_navigation/current_waypoint":
            waypoint_events.append(WaypointEvent(time_s, int(message.data)))
        elif topic == "/waypoint_navigation/status":
            status_events.append(StatusEvent(time_s, str(message.data)))

    if first_bag_ns is None:
        raise RuntimeError(f"bag contains no messages: {bag_path}")

    # route_input is the navigator's authoritative input. Older recordings may
    # only contain waypoint_path, so use it as a fallback.
    preferred = [item for item in route_messages if item[2] == ROUTE_TOPICS[0]]
    selected_routes = preferred or [
        item for item in route_messages if item[2] == ROUTE_TOPICS[1]
    ]
    route_snapshots = []
    for time_s, message, _topic in selected_routes:
        route_snapshots.append(
            RouteSnapshot(
                time_s=time_s,
                waypoints=tuple(waypoint_from_pose(pose) for pose in message.poses),
            )
        )
    route_snapshots.sort(key=lambda route: route.time_s)
    return (
        first_bag_ns * 1.0e-9,
        odometry,
        sorted(map_odom, key=lambda sample: sample.time_s),
        route_snapshots,
        sorted(waypoint_events, key=lambda event: event.time_s),
        sorted(status_events, key=lambda event: event.time_s),
        topic_types,
    )


def deduplicate_waypoint_events(
    events: Iterable[WaypointEvent],
    start_s: float,
    end_s: float,
) -> list[WaypointEvent]:
    selected = [
        event
        for event in events
        if start_s - 0.05 <= event.time_s < end_s + 0.05
    ]
    result: list[WaypointEvent] = []
    for event in selected:
        if result and result[-1].index == event.index:
            continue
        result.append(event)
    return result


def state_at(status_events: list[StatusEvent], time_s: float) -> str:
    state = ""
    for event in status_events:
        if event.time_s > time_s:
            break
        state = event.state
    return state


def interpolate_pose(samples: list[PoseSample], time_s: float) -> Optional[PoseSample]:
    if not samples:
        return None
    if time_s < samples[0].time_s or time_s > samples[-1].time_s:
        return None
    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    index = int(np.searchsorted(times, time_s, side="left"))
    if index == 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before = samples[index - 1]
    after = samples[index]
    span = after.time_s - before.time_s
    if span <= 1.0e-12:
        return before
    fraction = max(0.0, min(1.0, (time_s - before.time_s) / span))
    yaw_delta = wrap_angle(after.yaw - before.yaw)
    return PoseSample(
        time_s=time_s,
        x=before.x + fraction * (after.x - before.x),
        y=before.y + fraction * (after.y - before.y),
        yaw=wrap_angle(before.yaw + fraction * yaw_delta),
        vx=before.vx + fraction * (after.vx - before.vx),
        vy=before.vy + fraction * (after.vy - before.vy),
        wz=before.wz + fraction * (after.wz - before.wz),
    )


def pose_at_time_or_recent_before(
    samples: list[PoseSample], time_s: float, max_age_s: float = 0.25
) -> Optional[PoseSample]:
    pose = interpolate_pose(samples, time_s)
    if pose is not None:
        return pose
    if not samples or time_s < samples[0].time_s:
        return None
    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    index = int(np.searchsorted(times, time_s, side="right")) - 1
    if index < 0:
        return None
    sample = samples[index]
    if 0.0 <= time_s - sample.time_s <= max_age_s:
        return sample
    return None


def interpolate_transform(
    samples: list[TransformSample], time_s: float
) -> Optional[TransformSample]:
    if not samples:
        return None
    if time_s < samples[0].time_s or time_s > samples[-1].time_s:
        return None
    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    index = int(np.searchsorted(times, time_s, side="left"))
    if index == 0:
        return samples[0]
    if index >= len(samples):
        return samples[-1]
    before = samples[index - 1]
    after = samples[index]
    span = after.time_s - before.time_s
    if span <= 1.0e-12:
        return before
    fraction = max(0.0, min(1.0, (time_s - before.time_s) / span))
    return TransformSample(
        time_s=time_s,
        x=before.x + fraction * (after.x - before.x),
        y=before.y + fraction * (after.y - before.y),
        yaw=wrap_angle(before.yaw + fraction * wrap_angle(after.yaw - before.yaw)),
    )


def transform_odom_samples_to_map(
    samples: list[PoseSample],
    map_odom: list[TransformSample],
) -> list[PoseSample]:
    """Apply the recorded map->odom transform to odom-frame odometry."""
    transformed: list[PoseSample] = []
    for sample in samples:
        transform = interpolate_transform(map_odom, sample.time_s)
        if transform is None:
            continue
        cos_yaw = math.cos(transform.yaw)
        sin_yaw = math.sin(transform.yaw)
        transformed.append(
            PoseSample(
                time_s=sample.time_s,
                x=transform.x + cos_yaw * sample.x - sin_yaw * sample.y,
                y=transform.y + sin_yaw * sample.x + cos_yaw * sample.y,
                yaw=wrap_angle(sample.yaw + transform.yaw),
                vx=sample.vx,
                vy=sample.vy,
                wz=sample.wz,
            )
        )
    return transformed


def align_samples(
    samples: list[PoseSample],
    route_start_s: float,
    route_start: Waypoint,
    route_end_s: float,
) -> list[PoseSample]:
    anchor = interpolate_pose(samples, route_start_s)
    if anchor is None:
        anchor = next(
            (sample for sample in samples if sample.time_s >= route_start_s), None
        )
    if anchor is None:
        return []
    # All samples passed here are already expressed in map coordinates.  Align
    # only their origin to the route start.  A route waypoint yaw describes the
    # outgoing path, not the chassis yaw at the instant the route is accepted;
    # using it here rotates an entire route when the first action is a turn.
    delta_x = route_start.x - anchor.x
    delta_y = route_start.y - anchor.y
    aligned: list[PoseSample] = []
    for sample in samples:
        if sample.time_s < route_start_s - 1.0e-6:
            continue
        if sample.time_s > route_end_s + 1.0e-6:
            break
        aligned.append(
            PoseSample(
                time_s=sample.time_s,
                x=sample.x + delta_x,
                y=sample.y + delta_y,
                yaw=sample.yaw,
                vx=sample.vx,
                vy=sample.vy,
                wz=sample.wz,
            )
        )
    return aligned


def samples_for_interval(
    samples: list[PoseSample], start_s: float, end_s: float
) -> list[PoseSample]:
    if end_s < start_s:
        return []
    start = interpolate_pose(samples, start_s)
    end = interpolate_pose(samples, end_s)
    inside = [
        sample
        for sample in samples
        if start_s < sample.time_s < end_s
    ]
    result: list[PoseSample] = []
    if start is not None:
        result.append(start)
    result.extend(inside)
    if end is not None and (not result or end.time_s > result[-1].time_s):
        result.append(end)
    return result


def polyline_length(samples: list[PoseSample]) -> float:
    if len(samples) < 2:
        return float("nan")
    return float(
        sum(
            math.hypot(after.x - before.x, after.y - before.y)
            for before, after in zip(samples, samples[1:])
        )
    )


def interval_length_by_state(
    samples: list[PoseSample],
    status_events: list[StatusEvent],
    accepted_states: set[str],
) -> float:
    if len(samples) < 2:
        return float("nan")
    total = 0.0
    for before, after in zip(samples, samples[1:]):
        state = state_at(status_events, (before.time_s + after.time_s) * 0.5)
        if state in accepted_states:
            total += math.hypot(after.x - before.x, after.y - before.y)
    return total


def unwrapped_yaws(samples: list[PoseSample]) -> list[float]:
    if not samples:
        return []
    result = [samples[0].yaw]
    for before, after in zip(samples, samples[1:]):
        result.append(result[-1] + wrap_angle(after.yaw - before.yaw))
    return result


def turn_center_metrics(
    samples: list[PoseSample],
    start_s: float,
    end_s: float,
    configured_offset_m: float,
) -> Optional[dict[str, float]]:
    segment = samples_for_interval(samples, start_s, end_s)
    if len(segment) < 3:
        return None
    yaws = unwrapped_yaws(segment)
    first = segment[0]
    cos_origin = math.cos(-first.yaw)
    sin_origin = math.sin(-first.yaw)
    numerator = 0.0
    denominator = 0.0
    relative_points: list[tuple[float, float, float]] = []
    base_path = 0.0
    axle_path = 0.0
    axle_points: list[tuple[float, float]] = []
    for index, pose in enumerate(segment):
        dx = pose.x - first.x
        dy = pose.y - first.y
        body_x = cos_origin * dx - sin_origin * dy
        body_y = sin_origin * dx + cos_origin * dy
        theta = yaws[index] - yaws[0]
        basis_x = math.cos(theta) - 1.0
        basis_y = math.sin(theta)
        numerator += body_x * basis_x + body_y * basis_y
        denominator += basis_x * basis_x + basis_y * basis_y
        relative_points.append((body_x, body_y, theta))

        axle_x = pose.x - configured_offset_m * math.cos(pose.yaw)
        axle_y = pose.y - configured_offset_m * math.sin(pose.yaw)
        axle_points.append((axle_x, axle_y))
        if index > 0:
            previous_pose = segment[index - 1]
            base_path += math.hypot(
                pose.x - previous_pose.x,
                pose.y - previous_pose.y,
            )
            previous_axle = axle_points[index - 1]
            axle_path += math.hypot(axle_x - previous_axle[0], axle_y - previous_axle[1])

    fitted_offset = numerator / denominator if denominator > 1.0e-12 else float("nan")
    residual_sum = 0.0
    for body_x, body_y, theta in relative_points:
        expected_x = fitted_offset * (math.cos(theta) - 1.0)
        expected_y = fitted_offset * math.sin(theta)
        residual_sum += (body_x - expected_x) ** 2 + (body_y - expected_y) ** 2
    residual_rms = math.sqrt(residual_sum / max(1, len(relative_points)))

    last = segment[-1]
    body_dx = cos_origin * (last.x - first.x) - sin_origin * (last.y - first.y)
    body_dy = sin_origin * (last.x - first.x) + cos_origin * (last.y - first.y)
    yaw_delta = yaws[-1] - yaws[0]
    axle_net = math.hypot(
        axle_points[-1][0] - axle_points[0][0],
        axle_points[-1][1] - axle_points[0][1],
    )
    expected_chord = 2.0 * configured_offset_m * abs(math.sin(0.5 * yaw_delta))
    return {
        "sample_count": float(len(segment)),
        "duration_s": end_s - start_s,
        "yaw_delta_rad": yaw_delta,
        "body_delta_x_m": body_dx,
        "body_delta_y_m": body_dy,
        "base_path_m": base_path,
        "expected_base_chord_m": expected_chord,
        "fitted_center_offset_x_m": fitted_offset,
        "configured_center_offset_x_m": configured_offset_m,
        "center_offset_error_m": fitted_offset - configured_offset_m,
        "fit_residual_rms_m": residual_rms,
        "configured_axle_net_m": axle_net,
        "configured_axle_path_m": axle_path,
    }


def build_turn_center_rows(
    status_events: list[StatusEvent],
    routes: list[RouteSnapshot],
    route_end_times: list[float],
    odometry: dict[str, list[PoseSample]],
    configured_offset_m: float,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    turn_index = 0
    for index, event in enumerate(status_events):
        if event.state not in TURN_STATES:
            continue
        end_s = (
            status_events[index + 1].time_s
            if index + 1 < len(status_events)
            else event.time_s
        )
        if end_s - event.time_s < 0.5:
            continue
        turn_index += 1
        route_index = next(
            (
                idx + 1
                for idx, route in enumerate(routes)
                if route.time_s - 0.05 <= event.time_s < route_end_times[idx] + 0.05
            ),
            0,
        )
        for topic in PRIMARY_ODOMETRY_TOPICS:
            metrics = turn_center_metrics(
                odometry.get(topic, []),
                event.time_s,
                end_s,
                configured_offset_m,
            )
            if metrics is None:
                continue
            row: dict[str, Any] = {
                "turn_index": turn_index,
                "route": route_index,
                "start_time_s": event.time_s,
                "end_time_s": end_s,
                "topic": topic,
            }
            row.update(metrics)
            rows.append(row)
    return rows


def waypoint_requires_stop(
    route: RouteSnapshot,
    target_index: int,
    turn_threshold_rad: float = PRE_TURN_STOP_HEADING_THRESHOLD_RAD,
) -> bool:
    if target_index + 1 >= len(route.waypoints):
        return True
    current_start = route.waypoints[target_index - 1]
    current_target = route.waypoints[target_index]
    next_target = route.waypoints[target_index + 1]
    current_dx = current_target.x - current_start.x
    current_dy = current_target.y - current_start.y
    current_length = math.hypot(current_dx, current_dy)
    next_dx = next_target.x - current_target.x
    next_dy = next_target.y - current_target.y
    next_length = math.hypot(next_dx, next_dy)
    if current_length <= 1.0e-9 or next_length <= 1.0e-9:
        return True
    current_heading = math.atan2(current_dy, current_dx)
    next_heading = math.atan2(next_dy, next_dx)
    return abs(wrap_angle(next_heading - current_heading)) >= max(0.0, turn_threshold_rad)


def project_pose_to_segment(
    segment_start: Waypoint,
    segment_target: Waypoint,
    pose: PoseSample,
) -> dict[str, float]:
    length = math.hypot(
        segment_target.x - segment_start.x, segment_target.y - segment_start.y
    )
    if length <= 1.0e-9:
        return {
            "planned_distance_m": length,
            "progress_m": float("nan"),
            "remaining_m": float("nan"),
            "cross_track_m": float("nan"),
            "distance_to_target_m": math.hypot(
                pose.x - segment_target.x, pose.y - segment_target.y
            ),
        }
    unit_x = (segment_target.x - segment_start.x) / length
    unit_y = (segment_target.y - segment_start.y) / length
    from_start_x = pose.x - segment_start.x
    from_start_y = pose.y - segment_start.y
    progress = unit_x * from_start_x + unit_y * from_start_y
    cross = unit_x * from_start_y - unit_y * from_start_x
    return {
        "planned_distance_m": length,
        "progress_m": progress,
        "remaining_m": length - progress,
        "cross_track_m": cross,
        "distance_to_target_m": math.hypot(
            pose.x - segment_target.x, pose.y - segment_target.y
        ),
    }


def endpoint_metrics(
    segment_start: Waypoint,
    segment_target: Waypoint,
    samples: list[PoseSample],
) -> dict[str, float]:
    length = math.hypot(
        segment_target.x - segment_start.x, segment_target.y - segment_start.y
    )
    if not samples:
        return {
            "planned_distance_m": length,
            "start_pose_error_m": float("nan"),
            "along_displacement_m": float("nan"),
            "net_displacement_m": float("nan"),
            "trajectory_length_m": float("nan"),
            "drive_polyline_m": float("nan"),
            "turn_polyline_m": float("nan"),
            "brake_polyline_m": float("nan"),
            "other_polyline_m": float("nan"),
            "end_x_m": float("nan"),
            "end_y_m": float("nan"),
            "end_error_m": float("nan"),
            "end_cross_track_m": float("nan"),
            "end_remaining_m": float("nan"),
            "max_abs_cross_track_m": float("nan"),
        }
    first = samples[0]
    last = samples[-1]
    heading = math.atan2(
        segment_target.y - segment_start.y, segment_target.x - segment_start.x
    ) if length > 1.0e-9 else 0.0
    dx = last.x - first.x
    dy = last.y - first.y
    along = math.cos(heading) * dx + math.sin(heading) * dy
    net = math.hypot(dx, dy)
    projection = math.cos(heading) * (last.x - segment_start.x) + math.sin(
        heading
    ) * (last.y - segment_start.y)
    cross = math.cos(heading) * (last.y - segment_start.y) - math.sin(
        heading
    ) * (last.x - segment_start.x)
    max_cross = max(
        abs(
            math.cos(heading) * (sample.y - segment_start.y)
            - math.sin(heading) * (sample.x - segment_start.x)
        )
        for sample in samples
    )
    return {
        "planned_distance_m": length,
        "start_pose_error_m": math.hypot(
            first.x - segment_start.x, first.y - segment_start.y
        ),
        "along_displacement_m": along,
        "net_displacement_m": net,
        "trajectory_length_m": polyline_length(samples),
        "drive_polyline_m": float("nan"),
        "turn_polyline_m": float("nan"),
        "brake_polyline_m": float("nan"),
        "other_polyline_m": float("nan"),
        "end_x_m": last.x,
        "end_y_m": last.y,
        "end_error_m": math.hypot(
            last.x - segment_target.x, last.y - segment_target.y
        ),
        "end_cross_track_m": cross,
        "end_remaining_m": length - projection,
        "max_abs_cross_track_m": max_cross,
    }


def build_segments(
    route_index: int,
    route: RouteSnapshot,
    next_route_time: float,
    waypoint_events: list[WaypointEvent],
    status_events: list[StatusEvent],
    goal_times: list[float],
    odometry: dict[str, list[PoseSample]],
) -> tuple[list[dict[str, Any]], dict[str, list[PoseSample]]]:
    route_events = deduplicate_waypoint_events(
        waypoint_events, route.time_s, next_route_time
    )
    if not route_events or route_events[0].index != 0:
        route_events.insert(0, WaypointEvent(route.time_s, 0))

    # The status stream before the next route belongs to this route. Keeping
    # it as a full list also lets state_at() handle the initial route state.
    route_status = [
        event
        for event in status_events
        if route.time_s - 0.05 <= event.time_s <= next_route_time + 0.05
    ]
    route_goal_times = [
        event.time_s
        for event in route_status
        if event.state == "GOAL_REACHED"
    ]
    if goal_times:
        route_goal_times.extend(goal_times)
    route_goal_times = sorted(route_goal_times)

    aligned_by_topic: dict[str, list[PoseSample]] = {}
    for topic, samples in odometry.items():
        aligned_by_topic[topic] = align_samples(
            samples,
            route.time_s,
            route.waypoints[0],
            next_route_time,
        )

    rows: list[dict[str, Any]] = []
    n_waypoints = len(route.waypoints)
    for target_index in range(1, n_waypoints):
        start_event = next(
            (
                event
                for event in route_events
                if event.index == target_index
            ),
            None,
        )
        if start_event is None:
            continue
        end_event = next(
            (
                event
                for event in route_events
                if event.index == target_index + 1
                and event.time_s > start_event.time_s
            ),
            None,
        )
        if end_event is None and target_index == n_waypoints - 1:
            end_event = next(
                (
                    WaypointEvent(goal_time, n_waypoints)
                    for goal_time in route_goal_times
                    if goal_time > start_event.time_s
                ),
                None,
            )
        if end_event is None:
            continue
        segment_start = route.waypoints[target_index - 1]
        segment_target = route.waypoints[target_index]
        segment_start_s = start_event.time_s
        segment_end_s = min(end_event.time_s, next_route_time)
        is_micro = math.hypot(
            segment_target.x - segment_start.x,
            segment_target.y - segment_start.y,
        ) < 0.05
        for topic, aligned in aligned_by_topic.items():
            segment_samples = samples_for_interval(
                aligned, segment_start_s, segment_end_s
            )
            metrics = endpoint_metrics(segment_start, segment_target, segment_samples)
            metrics["trajectory_length_m"] = polyline_length(segment_samples)
            metrics["drive_polyline_m"] = interval_length_by_state(
                segment_samples, route_status, DRIVE_STATES
            )
            metrics["turn_polyline_m"] = interval_length_by_state(
                segment_samples, route_status, TURN_STATES
            )
            metrics["brake_polyline_m"] = interval_length_by_state(
                segment_samples, route_status, BRAKE_STATES
            )
            if math.isfinite(metrics["trajectory_length_m"]):
                known_length = sum(
                    value
                    for value in (
                        metrics["drive_polyline_m"],
                        metrics["turn_polyline_m"],
                        metrics["brake_polyline_m"],
                    )
                    if math.isfinite(value)
                )
                metrics["other_polyline_m"] = max(
                    0.0, metrics["trajectory_length_m"] - known_length
                )
            planned = metrics["planned_distance_m"]
            metrics["along_ratio"] = (
                metrics["along_displacement_m"] / planned
                if planned > 1.0e-6
                else float("nan")
            )
            metrics["trajectory_ratio"] = (
                metrics["trajectory_length_m"] / planned
                if planned > 1.0e-6
                else float("nan")
            )
            row: dict[str, Any] = {
                "route": route_index,
                "segment_target_index": target_index,
                "segment_label": f"{target_index - 1}->{target_index}",
                "is_micro_segment": is_micro,
                "start_time_s": segment_start_s,
                "end_time_s": segment_end_s,
                "duration_s": segment_end_s - segment_start_s,
                "status_at_start": state_at(route_status, segment_start_s),
                "start_x_m": segment_start.x,
                "start_y_m": segment_start.y,
                "target_x_m": segment_target.x,
                "target_y_m": segment_target.y,
                "topic": topic,
            }
            row.update(metrics)
            rows.append(row)
    return rows, aligned_by_topic


def waypoint_pass_rows(
    route_index: int,
    route: RouteSnapshot,
    next_route_time: float,
    waypoint_events: list[WaypointEvent],
    status_events: list[StatusEvent],
    aligned_by_topic: dict[str, list[PoseSample]],
) -> list[dict[str, Any]]:
    events = deduplicate_waypoint_events(
        waypoint_events, route.time_s, next_route_time
    )
    if not events or events[0].index != 0:
        events.insert(0, WaypointEvent(route.time_s, 0))
    route_status = [
        event
        for event in status_events
        if route.time_s - 0.05 <= event.time_s <= next_route_time + 0.05
    ]
    route_goal_times = [
        event.time_s for event in route_status if event.state == "GOAL_REACHED"
    ]

    rows: list[dict[str, Any]] = []
    n_waypoints = len(route.waypoints)
    for target_index in range(1, n_waypoints):
        start_event = next(
            (event for event in events if event.index == target_index),
            None,
        )
        if start_event is None:
            continue
        end_event = next(
            (
                event
                for event in events
                if event.index == target_index + 1
                and event.time_s > start_event.time_s
            ),
            None,
        )
        if end_event is None and target_index == n_waypoints - 1:
            end_event = next(
                (
                    WaypointEvent(goal_time, n_waypoints)
                    for goal_time in route_goal_times
                    if goal_time > start_event.time_s
                ),
                None,
            )
        if end_event is None:
            continue

        segment_start = route.waypoints[target_index - 1]
        segment_target = route.waypoints[target_index]
        final_waypoint = target_index + 1 == n_waypoints
        requires_stop = waypoint_requires_stop(route, target_index)
        for topic, samples in aligned_by_topic.items():
            pose = pose_at_time_or_recent_before(samples, end_event.time_s)
            if pose is None:
                continue
            projection = project_pose_to_segment(segment_start, segment_target, pose)
            distance = projection["distance_to_target_m"]
            remaining = projection["remaining_m"]
            cross = projection["cross_track_m"]
            radial_reached = (
                math.isfinite(distance) and distance <= WAYPOINT_TOLERANCE_M
            )
            longitudinal_passed = (
                math.isfinite(remaining)
                and remaining <= WAYPOINT_PASS_LONGITUDINAL_TOLERANCE_M
            )
            corridor_reached = (
                longitudinal_passed
                and math.isfinite(cross)
                and abs(cross)
                <= max(WAYPOINT_TOLERANCE_M, WAYPOINT_PASS_LATERAL_TOLERANCE_M)
            )
            continuous_passed = (
                not final_waypoint and not requires_stop and longitudinal_passed
            )
            would_advance = continuous_passed or radial_reached or corridor_reached
            rows.append(
                {
                    "route": route_index,
                    "segment_target_index": target_index,
                    "segment_label": f"{target_index - 1}->{target_index}",
                    "start_time_s": start_event.time_s,
                    "advance_time_s": end_event.time_s,
                    "duration_s": end_event.time_s - start_event.time_s,
                    "status_at_advance": state_at(route_status, end_event.time_s),
                    "topic": topic,
                    "final_waypoint": final_waypoint,
                    "requires_stop": requires_stop,
                    "turn_junction_marker": segment_target.stop_marker,
                    "planned_distance_m": projection["planned_distance_m"],
                    "progress_m": projection["progress_m"],
                    "remaining_m": remaining,
                    "cross_track_m": cross,
                    "distance_to_target_m": distance,
                    "radial_reached": radial_reached,
                    "longitudinal_passed": longitudinal_passed,
                    "corridor_reached": corridor_reached,
                    "continuous_passed": continuous_passed,
                    "would_advance_by_current_logic": would_advance,
                    "large_cross_track_continuous_pass": (
                        continuous_passed
                        and math.isfinite(cross)
                        and abs(cross)
                        > max(WAYPOINT_TOLERANCE_M, WAYPOINT_PASS_LATERAL_TOLERANCE_M)
                    ),
                }
            )
    return rows


def waypoint_rows(
    route_index: int,
    route: RouteSnapshot,
    next_route_time: float,
    waypoint_events: list[WaypointEvent],
    aligned_by_topic: dict[str, list[PoseSample]],
) -> list[dict[str, Any]]:
    events = deduplicate_waypoint_events(
        waypoint_events, route.time_s, next_route_time
    )
    rows: list[dict[str, Any]] = []
    for event in events:
        if event.index < 0 or event.index >= len(route.waypoints):
            continue
        target = route.waypoints[event.index]
        for topic, samples in aligned_by_topic.items():
            pose = interpolate_pose(samples, event.time_s)
            if pose is None:
                continue
            dx = pose.x - target.x
            dy = pose.y - target.y
            rows.append(
                {
                    "route": route_index,
                    "time_s": event.time_s,
                    "waypoint_index": event.index,
                    "topic": topic,
                    "target_x_m": target.x,
                    "target_y_m": target.y,
                    "actual_x_m": pose.x,
                    "actual_y_m": pose.y,
                    "distance_to_target_m": math.hypot(dx, dy),
                    "yaw_rad": pose.yaw,
                    "vx_mps": pose.vx,
                    "wz_radps": pose.wz,
                }
            )
    return rows


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def route_plot(
    route_index: int,
    route: RouteSnapshot,
    aligned_by_topic: dict[str, list[PoseSample]],
    output_path: Path,
) -> None:
    figure, axis = plt.subplots(figsize=(10, 8))
    route_x = [waypoint.x for waypoint in route.waypoints]
    route_y = [waypoint.y for waypoint in route.waypoints]
    axis.plot(
        route_x,
        route_y,
        "--",
        color="#dc2626",
        linewidth=2.2,
        marker="o",
        markersize=4,
        label="planned route",
        zorder=5,
    )
    for index, waypoint in enumerate(route.waypoints):
        axis.annotate(
            str(index),
            (waypoint.x, waypoint.y),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
            color="#991b1b",
        )
    for topic in PRIMARY_ODOMETRY_TOPICS:
        samples = aligned_by_topic.get(topic, [])
        if not samples:
            continue
        axis.plot(
            [sample.x for sample in samples],
            [sample.y for sample in samples],
            color=COLOR_BY_TOPIC[topic],
            linewidth=1.15 if topic != "/odometry/fused" else 1.8,
            alpha=0.85,
            label=LABEL_BY_TOPIC[topic],
        )
    axis.set_title(f"Route {route_index}: planned waypoints vs odometry")
    axis.set_xlabel("map x [m]")
    axis.set_ylabel("map y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def segment_plot(
    segment_rows: list[dict[str, Any]],
    output_path: Path,
    value_key: str,
    title: str,
    ylabel: str,
    include_micro: bool = True,
) -> None:
    routes = sorted({int(row["route"]) for row in segment_rows})
    figure, axes = plt.subplots(
        len(routes),
        1,
        figsize=(13, max(4.5, 4.0 * len(routes))),
        squeeze=False,
    )
    topic_order = list(PRIMARY_ODOMETRY_TOPICS)
    for axis, route_index in zip(axes[:, 0], routes):
        route_rows = [
            row
            for row in segment_rows
            if int(row["route"]) == route_index
            and (include_micro or not row["is_micro_segment"])
            and row["topic"] == "/odometry/fused"
        ]
        labels = []
        planned = []
        values_by_topic = {topic: [] for topic in topic_order}
        for row in route_rows:
            label = str(row["segment_label"])
            labels.append(label)
            planned.append(float(row["planned_distance_m"]))
            for topic in topic_order:
                match = next(
                    (
                        item
                        for item in segment_rows
                        if item["route"] == row["route"]
                        and item["segment_label"] == row["segment_label"]
                        and item["topic"] == topic
                    ),
                    None,
                )
                values_by_topic[topic].append(
                    float(match[value_key]) if match is not None else float("nan")
                )
        x = np.arange(len(labels), dtype=float)
        axis.plot(
            x,
            planned,
            color="#dc2626",
            marker="o",
            linewidth=2.0,
            label="planned",
        )
        for topic in topic_order:
            axis.plot(
                x,
                values_by_topic[topic],
                marker=".",
                linewidth=1.0,
                color=COLOR_BY_TOPIC[topic],
                label=LABEL_BY_TOPIC[topic],
            )
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=45, ha="right")
        axis.set_ylabel(ylabel)
        axis.set_title(f"Route {route_index}")
        axis.grid(True, alpha=0.25)
        if axis is axes[0, 0]:
            axis.legend(ncol=3, fontsize=8, loc="best")
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def error_plot(segment_rows: list[dict[str, Any]], output_path: Path) -> None:
    routes = sorted({int(row["route"]) for row in segment_rows})
    figure, axes = plt.subplots(
        len(routes),
        1,
        figsize=(13, max(4.5, 4.0 * len(routes))),
        squeeze=False,
    )
    for axis, route_index in zip(axes[:, 0], routes):
        route_rows = [
            row
            for row in segment_rows
            if int(row["route"]) == route_index
            and not row["is_micro_segment"]
            and row["topic"] in PRIMARY_ODOMETRY_TOPICS
        ]
        labels = sorted(
            {str(row["segment_label"]) for row in route_rows},
            key=lambda label: int(label.split("->")[0]),
        )
        x = np.arange(len(labels), dtype=float)
        for topic in PRIMARY_ODOMETRY_TOPICS:
            values = []
            for label in labels:
                match = next(
                    row
                    for row in route_rows
                    if row["segment_label"] == label and row["topic"] == topic
                )
                values.append(float(match["end_error_m"]))
            axis.plot(
                x,
                values,
                marker=".",
                linewidth=1.1,
                color=COLOR_BY_TOPIC[topic],
                label=LABEL_BY_TOPIC[topic],
            )
        axis.axhline(0.04, color="#dc2626", linestyle="--", linewidth=0.9)
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=45, ha="right")
        axis.set_ylabel("end error [m]")
        axis.set_title(f"Route {route_index}")
        axis.grid(True, alpha=0.25)
        if axis is axes[0, 0]:
            axis.legend(ncol=3, fontsize=8, loc="best")
    figure.suptitle("Endpoint error at each waypoint transition")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def fused_state_breakdown_plot(
    segment_rows: list[dict[str, Any]], output_path: Path
) -> None:
    routes = sorted({int(row["route"]) for row in segment_rows})
    figure, axes = plt.subplots(
        len(routes),
        1,
        figsize=(13, max(4.5, 4.0 * len(routes))),
        squeeze=False,
    )
    colors = {
        "drive": "#16a34a",
        "brake": "#f59e0b",
        "turn": "#7c3aed",
        "other": "#9ca3af",
    }
    for axis, route_index in zip(axes[:, 0], routes):
        route_rows = [
            row
            for row in segment_rows
            if int(row["route"]) == route_index
            and row["topic"] == "/odometry/fused"
            and not row["is_micro_segment"]
        ]
        labels = [str(row["segment_label"]) for row in route_rows]
        x = np.arange(len(labels), dtype=float)
        drive = np.asarray([float(row["drive_polyline_m"]) for row in route_rows])
        brake = np.asarray([float(row["brake_polyline_m"]) for row in route_rows])
        turn = np.asarray([float(row["turn_polyline_m"]) for row in route_rows])
        other = np.asarray([float(row["other_polyline_m"]) for row in route_rows])
        planned = [float(row["planned_distance_m"]) for row in route_rows]
        bottom = np.zeros(len(labels), dtype=float)
        for values, label in (
            (drive, "drive"),
            (brake, "brake"),
            (turn, "turn"),
            (other, "other"),
        ):
            axis.bar(
                x,
                values,
                bottom=bottom,
                width=0.72,
                color=colors[label],
                label=label,
            )
            bottom += np.nan_to_num(values, nan=0.0)
        axis.plot(
            x,
            planned,
            color="#dc2626",
            marker="o",
            linewidth=1.6,
            label="planned",
        )
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=45, ha="right")
        axis.set_ylabel("fused polyline [m]")
        axis.set_title(f"Route {route_index}")
        axis.grid(True, alpha=0.25, axis="y")
        if axis is axes[0, 0]:
            axis.legend(ncol=5, fontsize=8, loc="best")
    figure.suptitle("Fused segment distance split by navigation state")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def cross_track_plot(segment_rows: list[dict[str, Any]], output_path: Path) -> None:
    routes = sorted({int(row["route"]) for row in segment_rows})
    figure, axes = plt.subplots(
        len(routes),
        1,
        figsize=(13, max(4.5, 4.0 * len(routes))),
        squeeze=False,
    )
    for axis, route_index in zip(axes[:, 0], routes):
        route_rows = [
            row
            for row in segment_rows
            if int(row["route"]) == route_index
            and row["topic"] == "/odometry/fused"
            and not row["is_micro_segment"]
        ]
        labels = [str(row["segment_label"]) for row in route_rows]
        x = np.arange(len(labels), dtype=float)
        max_cross = [abs(float(row["max_abs_cross_track_m"])) for row in route_rows]
        end_cross = [abs(float(row["end_cross_track_m"])) for row in route_rows]
        axis.plot(
            x,
            max_cross,
            color="#7c3aed",
            marker="o",
            linewidth=1.4,
            label="max inside segment",
        )
        axis.plot(
            x,
            end_cross,
            color="#111827",
            marker=".",
            linewidth=1.1,
            label="at waypoint advance",
        )
        axis.axhline(
            WAYPOINT_PASS_LATERAL_TOLERANCE_M,
            color="#dc2626",
            linestyle="--",
            linewidth=0.9,
            label="pass lateral tolerance",
        )
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=45, ha="right")
        axis.set_ylabel("|cross-track| [m]")
        axis.set_title(f"Route {route_index}")
        axis.grid(True, alpha=0.25)
        if axis is axes[0, 0]:
            axis.legend(ncol=3, fontsize=8, loc="best")
    figure.suptitle("Fused cross-track error: segment max vs waypoint advance")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def pass_diagnostics_plot(
    pass_rows: list[dict[str, Any]], output_path: Path
) -> None:
    if not pass_rows:
        return
    routes = sorted({int(row["route"]) for row in pass_rows})
    figure, axes = plt.subplots(
        len(routes),
        1,
        figsize=(13, max(4.5, 4.0 * len(routes))),
        squeeze=False,
    )
    for axis, route_index in zip(axes[:, 0], routes):
        route_rows = [
            row
            for row in pass_rows
            if int(row["route"]) == route_index and row["topic"] == "/odometry/fused"
        ]
        labels = [str(row["segment_label"]) for row in route_rows]
        x = np.arange(len(labels), dtype=float)
        cross = [abs(float(row["cross_track_m"])) for row in route_rows]
        remaining = [float(row["remaining_m"]) for row in route_rows]
        distance = [float(row["distance_to_target_m"]) for row in route_rows]
        axis.plot(
            x,
            distance,
            color="#111827",
            marker="o",
            linewidth=1.3,
            label="distance to target",
        )
        axis.plot(
            x,
            cross,
            color="#7c3aed",
            marker=".",
            linewidth=1.1,
            label="abs cross-track",
        )
        axis.plot(
            x,
            remaining,
            color="#2563eb",
            marker=".",
            linewidth=1.1,
            label="remaining",
        )
        axis.axhline(WAYPOINT_TOLERANCE_M, color="#dc2626", linestyle="--", linewidth=0.9)
        axis.axhline(0.0, color="#6b7280", linewidth=0.8)
        axis.set_xticks(x)
        axis.set_xticklabels(labels, rotation=45, ha="right")
        axis.set_ylabel("advance metric [m]")
        axis.set_title(f"Route {route_index}")
        axis.grid(True, alpha=0.25)
        if axis is axes[0, 0]:
            axis.legend(ncol=4, fontsize=8, loc="best")
    figure.suptitle("Fused metrics at the exact waypoint-advance instant")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def turn_center_plot(turn_rows: list[dict[str, Any]], output_path: Path) -> None:
    if not turn_rows:
        return
    turns = sorted({int(row["turn_index"]) for row in turn_rows})
    figure, axes = plt.subplots(
        2,
        1,
        figsize=(14, 8),
        sharex=True,
        squeeze=False,
    )
    x = np.asarray(turns, dtype=float)
    configured = float(turn_rows[0]["configured_center_offset_x_m"])
    for topic in PRIMARY_ODOMETRY_TOPICS:
        topic_rows = {
            int(row["turn_index"]): row for row in turn_rows if row["topic"] == topic
        }
        offsets = [
            float(topic_rows[turn]["fitted_center_offset_x_m"])
            if turn in topic_rows
            else float("nan")
            for turn in turns
        ]
        residuals = [
            float(topic_rows[turn]["fit_residual_rms_m"])
            if turn in topic_rows
            else float("nan")
            for turn in turns
        ]
        axes[0, 0].plot(
            x,
            offsets,
            marker=".",
            linewidth=1.1,
            color=COLOR_BY_TOPIC[topic],
            label=LABEL_BY_TOPIC[topic],
        )
        axes[1, 0].plot(
            x,
            residuals,
            marker=".",
            linewidth=1.1,
            color=COLOR_BY_TOPIC[topic],
            label=LABEL_BY_TOPIC[topic],
        )
    axes[0, 0].axhline(
        configured,
        color="#dc2626",
        linestyle="--",
        linewidth=1.0,
        label=f"configured {configured:.3f} m",
    )
    axes[0, 0].axhline(0.0, color="#6b7280", linewidth=0.8)
    axes[0, 0].set_ylabel("fitted center offset [m]")
    axes[0, 0].set_title("Turn-center fit: positive offset means pivot behind base_link")
    axes[0, 0].set_ylim(configured - 0.12, configured + 0.12)
    axes[0, 0].grid(True, alpha=0.25)
    axes[0, 0].legend(ncol=4, fontsize=8, loc="best")
    axes[1, 0].set_ylabel("fit residual RMS [m]")
    axes[1, 0].set_xlabel("turn index")
    axes[1, 0].set_ylim(0.0, 0.15)
    axes[1, 0].grid(True, alpha=0.25)
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels([str(turn) for turn in turns], rotation=0)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160)
    plt.close(figure)


def compact_summary(
    routes: list[RouteSnapshot],
    segment_rows: list[dict[str, Any]],
    pass_rows: list[dict[str, Any]],
    output_dir: Path,
) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "route_count": len(routes),
        "routes": [],
        "output_dir": str(output_dir),
    }
    for route_index, route in enumerate(routes, start=1):
        planned_total = sum(
            math.hypot(
                after.x - before.x,
                after.y - before.y,
            )
            for before, after in zip(route.waypoints, route.waypoints[1:])
        )
        route_rows = [
            row
            for row in segment_rows
            if int(row["route"]) == route_index and row["topic"] == "/odometry/fused"
        ]
        pass_route_rows = [
            row
            for row in pass_rows
            if int(row["route"]) == route_index and row["topic"] == "/odometry/fused"
        ]
        summary["routes"].append(
            {
                "route": route_index,
                "waypoint_count": len(route.waypoints),
                "planned_polyline_m": planned_total,
                "fused_segment_count": len(route_rows),
                "fused_trajectory_sum_m": sum(
                    float(row["trajectory_length_m"])
                    for row in route_rows
                    if math.isfinite(float(row["trajectory_length_m"]))
                ),
                "fused_end_error_max_m": max(
                    (
                        float(row["end_error_m"])
                        for row in route_rows
                        if math.isfinite(float(row["end_error_m"]))
                    ),
                    default=float("nan"),
                ),
                "fused_max_abs_cross_track_m": max(
                    (
                        abs(float(row["max_abs_cross_track_m"]))
                        for row in route_rows
                        if math.isfinite(float(row["max_abs_cross_track_m"]))
                    ),
                    default=float("nan"),
                ),
                "fused_bad_continuous_pass_count": sum(
                    1
                    for row in pass_route_rows
                    if str(row["large_cross_track_continuous_pass"]) == "True"
                ),
            }
        )
    with (output_dir / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=True)
    return summary


def print_key_findings(segment_rows: list[dict[str, Any]]) -> None:
    print("\nSegment analysis")
    print("================")
    for route in sorted({int(row["route"]) for row in segment_rows}):
        print(f"\nRoute {route}")
        for label in sorted(
            {
                str(row["segment_label"])
                for row in segment_rows
                if int(row["route"]) == route
            },
            key=lambda item: int(item.split("->")[0]),
        ):
            fused = next(
                row
                for row in segment_rows
                if int(row["route"]) == route
                and row["segment_label"] == label
                and row["topic"] == "/odometry/fused"
            )
            print(
                f"  {label:>5} plan={float(fused['planned_distance_m']):6.3f} m "
                f"fused_path={float(fused['trajectory_length_m']):6.3f} m "
                f"along={float(fused['along_displacement_m']):6.3f} m "
                f"end_err={float(fused['end_error_m']):6.3f} m "
                f"cross={float(fused['end_cross_track_m']):+6.3f} m"
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        default="latest_navigation_bag",
        help="ROS 2 bag directory (default: latest_navigation_bag)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="directory for CSV/PNG output (default: analysis_bags/<bag-name>)",
    )
    parser.add_argument(
        "--turn-center-offset",
        type=float,
        default=DEFAULT_TURN_CENTER_OFFSET_M,
        help=(
            "configured base_link x offset from wheel axle midpoint in meters "
            f"(default: {DEFAULT_TURN_CENTER_OFFSET_M:.3f})"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    if not bag_path.exists():
        print(f"bag does not exist: {bag_path}", file=sys.stderr)
        return 2
    bag_name = bag_path.parent.name if bag_path.name == "bag" else bag_path.name
    output_dir = (
        Path(args.output_dir).expanduser()
        if args.output_dir
        else Path("analysis_bags") / bag_name
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    (
        _bag_start_time,
        odometry,
        map_odom,
        routes,
        waypoint_events,
        status_events,
        topic_types,
    ) = collect_bag(bag_path)
    if not routes:
        print("no route_input or waypoint_path messages found", file=sys.stderr)
        return 3

    # The planner and fused/visual streams use map.  Local EKF and wheel
    # odometry use odom, so transform them through the recorded TF before
    # comparing them with the route.
    map_odometry = dict(odometry)
    for topic in ("/odometry/local", "/wheel/odom"):
        map_odometry[topic] = transform_odom_samples_to_map(
            odometry[topic], map_odom
        )

    segment_rows: list[dict[str, Any]] = []
    waypoint_rows_all: list[dict[str, Any]] = []
    pass_rows_all: list[dict[str, Any]] = []
    aligned_routes: list[tuple[RouteSnapshot, dict[str, list[PoseSample]]]] = []
    route_end_times = [
        routes[index + 1].time_s if index + 1 < len(routes) else float("inf")
        for index in range(len(routes))
    ]
    for route_index, (route, route_end_time) in enumerate(
        zip(routes, route_end_times), start=1
    ):
        route_end = route_end_time
        if not math.isfinite(route_end):
            route_end = max(
                [event.time_s for event in status_events if event.time_s >= route.time_s]
                or [route.time_s + 1.0]
            )
        rows, aligned = build_segments(
            route_index,
            route,
            route_end,
            waypoint_events,
            status_events,
            [],
            map_odometry,
        )
        segment_rows.extend(rows)
        waypoint_rows_all.extend(
            waypoint_rows(
                route_index,
                route,
                route_end,
                waypoint_events,
                aligned,
            )
        )
        pass_rows_all.extend(
            waypoint_pass_rows(
                route_index,
                route,
                route_end,
                waypoint_events,
                status_events,
                aligned,
            )
        )
        aligned_routes.append((route, aligned))
        route_plot(
            route_index,
            route,
            aligned,
            output_dir / f"route_{route_index:02d}_trajectory.png",
        )

    turn_center_rows = build_turn_center_rows(
        status_events,
        routes,
        route_end_times,
        odometry,
        args.turn_center_offset,
    )
    write_csv(output_dir / "segments.csv", segment_rows)
    write_csv(output_dir / "waypoint_events.csv", waypoint_rows_all)
    write_csv(output_dir / "waypoint_pass_diagnostics.csv", pass_rows_all)
    write_csv(output_dir / "turn_center_diagnostics.csv", turn_center_rows)
    segment_plot(
        segment_rows,
        output_dir / "segment_distances.png",
        "trajectory_length_m",
        "Per-segment odometry polyline length vs planned length",
        "distance [m]",
    )
    segment_plot(
        segment_rows,
        output_dir / "segment_along_displacement.png",
        "along_displacement_m",
        "Per-segment signed displacement along the planned direction",
        "along displacement [m]",
    )
    segment_plot(
        segment_rows,
        output_dir / "segment_ratios.png",
        "trajectory_ratio",
        "Per-segment trajectory length ratio",
        "trajectory / planned",
        include_micro=False,
    )
    error_plot(segment_rows, output_dir / "waypoint_endpoint_error.png")
    fused_state_breakdown_plot(
        segment_rows, output_dir / "fused_state_distance_breakdown.png"
    )
    cross_track_plot(segment_rows, output_dir / "fused_cross_track_error.png")
    pass_diagnostics_plot(
        pass_rows_all, output_dir / "waypoint_pass_diagnostics.png"
    )
    turn_center_plot(turn_center_rows, output_dir / "turn_center_fit.png")
    compact_summary(routes, segment_rows, pass_rows_all, output_dir)
    with (output_dir / "bag_topics.json").open("w", encoding="utf-8") as stream:
        json.dump(sorted(topic_types), stream, indent=2, ensure_ascii=True)
    print_key_findings(segment_rows)
    print(f"\nWrote analysis to: {output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
