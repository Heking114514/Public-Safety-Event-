from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

from geometry_msgs.msg import Point32, Polygon, Pose
from mission_control_interfaces.msg import EventCandidate

from .events import EventData
from .model import PlanSpec
from .obstacles import Polygon as ObstaclePolygon


@dataclass(frozen=True)
class VehiclePose:
    x: float
    y: float
    yaw: float
    received_at: float

    def fresh(self, timeout_s: float) -> bool:
        return time.monotonic() - self.received_at <= timeout_s


@dataclass
class ActiveRoute:
    route_id: int
    spec: PlanSpec
    plan: Any
    acknowledged: bool = False
    motion_seen: bool = False


def quaternion_yaw(quaternion: Any) -> float | None:
    values = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    if not all(math.isfinite(value) for value in values):
        return None
    norm = sum(value * value for value in values)
    if norm <= 1.0e-12:
        return None
    x, y, z, w = (value / math.sqrt(norm) for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_message(pose: VehiclePose) -> Pose:
    message = Pose()
    message.position.x = pose.x
    message.position.y = pose.y
    message.orientation.z = math.sin(0.5 * pose.yaw)
    message.orientation.w = math.cos(0.5 * pose.yaw)
    return message


def polygon_message(points: ObstaclePolygon) -> Polygon:
    return Polygon(points=[Point32(x=float(x), y=float(y)) for x, y in points])


def event_data(message: EventCandidate) -> EventData | None:
    yaw = quaternion_yaw(message.pose.orientation)
    if yaw is None:
        return None
    return EventData(
        candidate_id=str(message.candidate_id),
        source=str(message.source),
        event_type=str(message.event_type),
        confidence=float(message.confidence),
        x=float(message.pose.position.x),
        y=float(message.pose.position.y),
        yaw=yaw,
        evidence_uri=str(message.evidence_uri),
        announcement=str(message.announcement),
    )


def event_message(event: EventData, stamp: Any) -> EventCandidate:
    message = EventCandidate()
    message.header.stamp = stamp
    message.header.frame_id = "map"
    message.candidate_id = event.candidate_id
    message.source = event.source
    message.event_type = event.event_type
    message.confidence = event.confidence
    message.pose.position.x = event.x
    message.pose.position.y = event.y
    message.pose.orientation.z = math.sin(0.5 * event.yaw)
    message.pose.orientation.w = math.cos(0.5 * event.yaw)
    message.evidence_uri = event.evidence_uri
    message.announcement = event.announcement
    return message
