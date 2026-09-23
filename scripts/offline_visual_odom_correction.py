#!/usr/bin/env python3
"""Offline visual/map correction for recorded fused odometry.

This script reads a rosbag2 sqlite database directly.  When raw image topics are
missing, it uses the recorded visual odometry stream as the visual observation
proxy and snaps fused odometry toward the known arena road-centre graph.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import Odometry, Path as PathMsg  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402


DEFAULT_BAG = Path("navigation_runs/20260921T150024.404580Z_bc4af70669a5/bag")
DEFAULT_MAP = Path("src/arena_path_planner/config/arena_map.yaml")


@dataclass(frozen=True)
class PoseSample:
    stamp: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class RoadSegment:
    ax: float
    ay: float
    bx: float
    by: float
    label: str


@dataclass
class CorrectedSample:
    stamp: float
    fused_x: float
    fused_y: float
    fused_yaw: float
    corrected_x: float
    corrected_y: float
    corrected_yaw: float
    visual_x: float | None
    visual_y: float | None
    visual_yaw: float | None
    offset_x: float
    offset_y: float
    offset_yaw: float
    residual_x: float
    residual_y: float
    residual_yaw: float
    matched_road: str
    road_error_fused: float
    road_error_corrected: float


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm <= 1.0e-12:
        return 0.0
    x = q.x / norm
    y = q.y / norm
    z = q.z / norm
    w = q.w / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def message_stamp_seconds(msg, fallback_ns: int) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None:
        value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if value > 0.0:
            return value
    return float(fallback_ns) * 1.0e-9


def bag_db_path(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3":
        return path
    candidates = sorted(path.glob("*.db3"))
    if not candidates:
        raise FileNotFoundError(f"no .db3 file found under {path}")
    return candidates[0]


def topic_table(db_path: Path) -> dict[str, tuple[int, str, int]]:
    with sqlite3.connect(db_path) as conn:
        rows = conn.execute(
            """
            select topics.id, topics.name, topics.type, count(messages.id)
            from topics left join messages on topics.id = messages.topic_id
            group by topics.id
            """
        ).fetchall()
    return {name: (topic_id, topic_type, count) for topic_id, name, topic_type, count in rows}


def iter_topic_messages(db_path: Path, topic: str):
    topics = topic_table(db_path)
    if topic not in topics:
        return
    topic_id = topics[topic][0]
    with sqlite3.connect(db_path) as conn:
        cursor = conn.execute(
            "select timestamp, data from messages where topic_id = ? order by timestamp",
            (topic_id,),
        )
        for timestamp_ns, data in cursor:
            yield int(timestamp_ns), data


def load_odometry(db_path: Path, topic: str) -> list[PoseSample]:
    samples: list[PoseSample] = []
    for timestamp_ns, data in iter_topic_messages(db_path, topic):
        msg = deserialize_message(data, Odometry)
        pose = msg.pose.pose
        if not all(
            math.isfinite(value)
            for value in (
                pose.position.x,
                pose.position.y,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
        ):
            continue
        samples.append(
            PoseSample(
                message_stamp_seconds(msg, timestamp_ns),
                float(pose.position.x),
                float(pose.position.y),
                yaw_from_quaternion(pose.orientation),
            )
        )
    return dedupe_sorted(samples)


def load_paths(db_path: Path, topics: Iterable[str]) -> list[list[PoseSample]]:
    paths: list[list[PoseSample]] = []
    for topic in topics:
        for timestamp_ns, data in iter_topic_messages(db_path, topic):
            msg = deserialize_message(data, PathMsg)
            path: list[PoseSample] = []
            for pose_stamped in msg.poses:
                pose = pose_stamped.pose
                path.append(
                    PoseSample(
                        message_stamp_seconds(msg, timestamp_ns),
                        float(pose.position.x),
                        float(pose.position.y),
                        yaw_from_quaternion(pose.orientation),
                    )
                )
            if len(path) >= 2:
                paths.append(path)
    return paths


def dedupe_sorted(samples: list[PoseSample]) -> list[PoseSample]:
    samples.sort(key=lambda sample: sample.stamp)
    result: list[PoseSample] = []
    for sample in samples:
        if result and sample.stamp <= result[-1].stamp:
            result[-1] = sample
        else:
            result.append(sample)
    return result


def interpolate_pose(samples: list[PoseSample], stamp: float, tolerance: float) -> PoseSample | None:
    if not samples:
        return None
    stamps = [sample.stamp for sample in samples]
    index = bisect.bisect_left(stamps, stamp)
    if index < len(samples) and abs(samples[index].stamp - stamp) <= tolerance:
        return samples[index]
    if index == 0:
        return samples[0] if abs(samples[0].stamp - stamp) <= tolerance else None
    if index >= len(samples):
        return samples[-1] if abs(samples[-1].stamp - stamp) <= tolerance else None
    before = samples[index - 1]
    after = samples[index]
    if stamp - before.stamp > tolerance or after.stamp - stamp > tolerance:
        return None
    span = after.stamp - before.stamp
    if span <= 0.0:
        return before
    ratio = (stamp - before.stamp) / span
    return PoseSample(
        stamp,
        before.x + (after.x - before.x) * ratio,
        before.y + (after.y - before.y) * ratio,
        wrap_angle(before.yaw + wrap_angle(after.yaw - before.yaw) * ratio),
    )


def load_road_segments(map_path: Path) -> list[RoadSegment]:
    with map_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream)

    start_x, start_y = root["start"]["position_m"]
    start_yaw = math.radians(float(root["start"]["heading_deg"]))
    cosine = math.cos(start_yaw)
    sine = math.sin(start_yaw)

    def arena_to_map(point: tuple[float, float]) -> tuple[float, float]:
        dx = float(point[0]) - float(start_x)
        dy = float(point[1]) - float(start_y)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    graph = root["inspection_graph"]
    nodes = [tuple(node) for node in graph["nodes"]]
    segments: list[RoadSegment] = []
    for edge in graph["edges"]:
        ax, ay = arena_to_map(nodes[int(edge[0])])
        bx, by = arena_to_map(nodes[int(edge[1])])
        segments.append(RoadSegment(ax, ay, bx, by, str(edge[2])))

    for junction in root.get("arena", {}).get("turn_junctions", []):
        ax, ay = arena_to_map((float(start_x), float(start_y)))
        bx, by = arena_to_map(tuple(junction))
        if not any(
            min(distance2((seg.ax, seg.ay), (ax, ay)), distance2((seg.bx, seg.by), (ax, ay)))
            < 1.0e-12
            and min(
                distance2((seg.ax, seg.ay), (bx, by)),
                distance2((seg.bx, seg.by), (bx, by)),
            )
            < 1.0e-12
            for seg in segments
        ):
            segments.append(RoadSegment(ax, ay, bx, by, "LAUNCH_SPUR"))
    return segments


def distance2(a: tuple[float, float], b: tuple[float, float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def nearest_on_segment(x: float, y: float, segment: RoadSegment) -> tuple[float, float, float]:
    dx = segment.bx - segment.ax
    dy = segment.by - segment.ay
    length2 = dx * dx + dy * dy
    if length2 <= 1.0e-12:
        return segment.ax, segment.ay, math.hypot(x - segment.ax, y - segment.ay)
    ratio = max(0.0, min(1.0, ((x - segment.ax) * dx + (y - segment.ay) * dy) / length2))
    px = segment.ax + ratio * dx
    py = segment.ay + ratio * dy
    return px, py, math.hypot(x - px, y - py)


def undirected_yaw_error(target: float, observed: float) -> float:
    error = wrap_angle(target - observed)
    if error > math.pi * 0.5:
        error -= math.pi
    elif error < -math.pi * 0.5:
        error += math.pi
    return wrap_angle(error)


def closest_road(
    pose: PoseSample,
    roads: list[RoadSegment],
    max_distance: float,
    max_angle: float,
) -> tuple[RoadSegment, float, float, float, float] | None:
    best = None
    best_score = float("inf")
    for road in roads:
        px, py, dist = nearest_on_segment(pose.x, pose.y, road)
        if dist > max_distance:
            continue
        yaw = math.atan2(road.by - road.ay, road.bx - road.ax)
        yaw_error = undirected_yaw_error(yaw, pose.yaw)
        if abs(yaw_error) > max_angle:
            continue
        score = dist + 0.05 * abs(yaw_error)
        if score < best_score:
            best_score = score
            best = (road, px, py, dist, yaw_error)
    return best


def clamp_vector(x: float, y: float, maximum: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= maximum or norm <= 1.0e-12:
        return x, y
    scale = maximum / norm
    return x * scale, y * scale


def corrected_pose(fused: PoseSample, ox: float, oy: float, oyaw: float) -> PoseSample:
    return PoseSample(fused.stamp, fused.x + ox, fused.y + oy, wrap_angle(fused.yaw + oyaw))


def correct_trajectory(
    fused: list[PoseSample],
    visual: list[PoseSample],
    roads: list[RoadSegment],
    *,
    sync_tolerance: float,
    correction_tau: float,
    max_match_distance: float,
    max_match_angle: float,
    max_step_m: float,
    max_step_yaw: float,
    max_total_offset_m: float,
    max_total_yaw: float,
) -> list[CorrectedSample]:
    output: list[CorrectedSample] = []
    offset_x = 0.0
    offset_y = 0.0
    offset_yaw = 0.0
    last_update_stamp: float | None = None

    for sample in fused:
        visual_pose = interpolate_pose(visual, sample.stamp, sync_tolerance)
        current = corrected_pose(sample, offset_x, offset_y, offset_yaw)
        road_match = closest_road(current, roads, max_match_distance, max_match_angle)
        visual_match = (
            closest_road(visual_pose, roads, max_match_distance, max_match_angle)
            if visual_pose is not None
            else None
        )
        if (
            road_match is not None
            and visual_match is not None
            and road_match[0].label != visual_match[0].label
        ):
            road_match = None

        residual_x = 0.0
        residual_y = 0.0
        residual_yaw = 0.0
        road_label = ""
        if road_match is not None:
            road, _vx, _vy, _vdist, _vyaw_error = road_match
            target_x, target_y, _ = nearest_on_segment(current.x, current.y, road)
            road_yaw = math.atan2(road.by - road.ay, road.bx - road.ax)
            residual_x = target_x - current.x
            residual_y = target_y - current.y
            residual_yaw = undirected_yaw_error(road_yaw, current.yaw)
            road_label = road.label

            dt = 0.0 if last_update_stamp is None else max(0.0, sample.stamp - last_update_stamp)
            alpha = 1.0 if last_update_stamp is None else 1.0 - math.exp(-dt / correction_tau)
            step_x, step_y = clamp_vector(residual_x * alpha, residual_y * alpha, max_step_m)
            step_yaw = max(-max_step_yaw, min(max_step_yaw, residual_yaw * alpha))
            offset_x += step_x
            offset_y += step_y
            offset_yaw = wrap_angle(offset_yaw + step_yaw)
            offset_x, offset_y = clamp_vector(offset_x, offset_y, max_total_offset_m)
            offset_yaw = max(-max_total_yaw, min(max_total_yaw, offset_yaw))
            last_update_stamp = sample.stamp

        final = corrected_pose(sample, offset_x, offset_y, offset_yaw)
        fused_road_error = closest_distance_to_roads(sample.x, sample.y, roads)
        corrected_road_error = closest_distance_to_roads(final.x, final.y, roads)
        output.append(
            CorrectedSample(
                stamp=sample.stamp,
                fused_x=sample.x,
                fused_y=sample.y,
                fused_yaw=sample.yaw,
                corrected_x=final.x,
                corrected_y=final.y,
                corrected_yaw=final.yaw,
                visual_x=None if visual_pose is None else visual_pose.x,
                visual_y=None if visual_pose is None else visual_pose.y,
                visual_yaw=None if visual_pose is None else visual_pose.yaw,
                offset_x=offset_x,
                offset_y=offset_y,
                offset_yaw=offset_yaw,
                residual_x=residual_x,
                residual_y=residual_y,
                residual_yaw=residual_yaw,
                matched_road=road_label,
                road_error_fused=fused_road_error,
                road_error_corrected=corrected_road_error,
            )
        )
    return output


def closest_distance_to_roads(x: float, y: float, roads: list[RoadSegment]) -> float:
    if not roads:
        return float("nan")
    return min(nearest_on_segment(x, y, road)[2] for road in roads)


def write_csv(path: Path, samples: list[CorrectedSample]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(CorrectedSample.__dataclass_fields__))
        writer.writeheader()
        for sample in samples:
            writer.writerow(sample.__dict__)


def mean(values: list[float]) -> float:
    values = [value for value in values if math.isfinite(value)]
    return sum(values) / len(values) if values else float("nan")


def percentile(values: list[float], p: float) -> float:
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return float("nan")
    index = int(round((len(values) - 1) * p))
    return values[index]


def write_summary(path: Path, samples: list[CorrectedSample], topics: dict[str, tuple[int, str, int]]) -> None:
    fused_errors = [sample.road_error_fused for sample in samples]
    corrected_errors = [sample.road_error_corrected for sample in samples]
    matched = [sample for sample in samples if sample.matched_road]
    image_topics = {
        name: count
        for name, (_topic_id, topic_type, count) in topics.items()
        if topic_type == "sensor_msgs/msg/Image" or "image" in name
    }
    summary = {
        "sample_count": len(samples),
        "matched_sample_count": len(matched),
        "image_topics": image_topics,
        "mean_road_error_fused_m": mean(fused_errors),
        "mean_road_error_corrected_m": mean(corrected_errors),
        "p95_road_error_fused_m": percentile(fused_errors, 0.95),
        "p95_road_error_corrected_m": percentile(corrected_errors, 0.95),
        "final_offset_m": {
            "x": samples[-1].offset_x if samples else 0.0,
            "y": samples[-1].offset_y if samples else 0.0,
            "yaw": samples[-1].offset_yaw if samples else 0.0,
        },
        "note": (
            "No raw image messages were found; correction used recorded visual "
            "odometry as a visual-observation proxy."
            if not image_topics
            else "Raw image topics exist; this script version still uses visual odometry proxy."
        ),
    }
    path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def plot_trajectory(
    path: Path,
    samples: list[CorrectedSample],
    roads: list[RoadSegment],
    planned_paths: list[list[PoseSample]],
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    for road in roads:
        ax.plot([road.ax, road.bx], [road.ay, road.by], color="#d1d5db", linewidth=1.2)
    for route in planned_paths:
        ax.plot(
            [pose.x for pose in route],
            [pose.y for pose in route],
            color="#f59e0b",
            linewidth=1.5,
            alpha=0.8,
            label="planned route",
        )
    ax.plot([s.fused_x for s in samples], [s.fused_y for s in samples], color="#ef4444", label="fused")
    ax.plot(
        [s.corrected_x for s in samples],
        [s.corrected_y for s in samples],
        color="#2563eb",
        label="corrected",
    )
    visual_points = [s for s in samples if s.visual_x is not None and s.visual_y is not None]
    if visual_points:
        ax.plot(
            [s.visual_x for s in visual_points],
            [s.visual_y for s in visual_points],
            color="#16a34a",
            linewidth=1.0,
            alpha=0.75,
            label="visual odom",
        )
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3)
    ax.set_title("Fused vs offline visual/map corrected odometry")
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_errors(path: Path, samples: list[CorrectedSample]) -> None:
    if not samples:
        return
    t0 = samples[0].stamp
    times = [sample.stamp - t0 for sample in samples]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(times, [s.road_error_fused for s in samples], label="fused", color="#ef4444")
    axes[0].plot(
        times,
        [s.road_error_corrected for s in samples],
        label="corrected",
        color="#2563eb",
    )
    axes[0].set_ylabel("road error [m]")
    axes[0].legend()
    axes[0].grid(True, linewidth=0.3)

    axes[1].plot(times, [s.offset_x for s in samples], label="offset x")
    axes[1].plot(times, [s.offset_y for s in samples], label="offset y")
    axes[1].set_ylabel("offset [m]")
    axes[1].legend()
    axes[1].grid(True, linewidth=0.3)

    axes[2].plot(times, [s.offset_yaw for s in samples], label="offset yaw", color="#7c3aed")
    axes[2].set_ylabel("yaw [rad]")
    axes[2].set_xlabel("time [s]")
    axes[2].legend()
    axes[2].grid(True, linewidth=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--fused-topic", default="/odometry/fused")
    parser.add_argument("--visual-topic", default="/odometry/visual_continuous")
    parser.add_argument("--sync-tolerance", type=float, default=0.15)
    parser.add_argument("--correction-tau", type=float, default=2.5)
    parser.add_argument("--max-match-distance", type=float, default=0.08)
    parser.add_argument("--max-match-angle", type=float, default=0.45)
    parser.add_argument("--max-step-m", type=float, default=0.0025)
    parser.add_argument("--max-step-yaw", type=float, default=0.0)
    parser.add_argument("--max-total-offset-m", type=float, default=0.12)
    parser.add_argument("--max-total-yaw", type=float, default=0.08)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    db_path = bag_db_path(args.bag)
    out_dir = args.out
    if out_dir is None:
        out_dir = Path("analysis_bags") / (args.bag.parent.name if args.bag.name == "bag" else args.bag.stem)
        out_dir = out_dir / "offline_visual_correct"
    out_dir.mkdir(parents=True, exist_ok=True)

    topics = topic_table(db_path)
    fused = load_odometry(db_path, args.fused_topic)
    visual = load_odometry(db_path, args.visual_topic)
    if not visual:
        visual = load_odometry(db_path, "/odometry/visual_raw")
    if not fused:
        raise RuntimeError(f"no fused odometry samples found on {args.fused_topic}")
    if not visual:
        raise RuntimeError("no visual odometry proxy samples found")

    roads = load_road_segments(args.map)
    planned_paths = load_paths(
        db_path,
        [
            "/arena_path_planner/navigation_path",
            "/waypoint_path",
            "/waypoint_navigation/route_input",
        ],
    )
    samples = correct_trajectory(
        fused,
        visual,
        roads,
        sync_tolerance=args.sync_tolerance,
        correction_tau=args.correction_tau,
        max_match_distance=args.max_match_distance,
        max_match_angle=args.max_match_angle,
        max_step_m=args.max_step_m,
        max_step_yaw=args.max_step_yaw,
        max_total_offset_m=args.max_total_offset_m,
        max_total_yaw=args.max_total_yaw,
    )

    write_csv(out_dir / "corrected_odom.csv", samples)
    write_summary(out_dir / "summary.json", samples, topics)
    plot_trajectory(out_dir / "trajectory_corrected.png", samples, roads, planned_paths)
    plot_errors(out_dir / "correction_timeseries.png", samples)

    summary = json.loads((out_dir / "summary.json").read_text(encoding="utf-8"))
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"wrote {out_dir}")


if __name__ == "__main__":
    main()
