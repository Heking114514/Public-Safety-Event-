from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Optional


@dataclass(frozen=True)
class EventData:
    candidate_id: str
    source: str
    event_type: str
    confidence: float
    x: float
    y: float
    yaw: float
    evidence_uri: str = ""
    announcement: str = ""
    attempts: int = 0


class EventRegistry:
    def __init__(self, confidence_threshold: float, merge_distance: float) -> None:
        self.confidence_threshold = confidence_threshold
        self.merge_distance = merge_distance
        self._pending: list[EventData] = []
        self._reported: list[EventData] = []

    def offer(self, event: EventData) -> bool:
        if not self._valid(event) or self._duplicate(event):
            return False
        self._pending.append(event)
        return True

    def current(self) -> Optional[EventData]:
        return self._pending[0] if self._pending else None

    def mark_reported(self) -> EventData:
        if not self._pending:
            raise RuntimeError("no pending event to confirm")
        event = self._pending.pop(0)
        self._reported.append(event)
        return event

    def mark_attempt(self) -> EventData:
        if not self._pending:
            raise RuntimeError("no pending event to retry")
        event = replace(self._pending[0], attempts=self._pending[0].attempts + 1)
        self._pending[0] = event
        return event

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def reported_count(self) -> int:
        return len(self._reported)

    def reset(self) -> None:
        self._pending.clear()
        self._reported.clear()

    def _valid(self, event: EventData) -> bool:
        return (
            bool(event.event_type.strip())
            and math.isfinite(event.confidence)
            and event.confidence >= self.confidence_threshold
            and all(math.isfinite(value) for value in (event.x, event.y, event.yaw))
        )

    def _duplicate(self, event: EventData) -> bool:
        for existing in self._pending + self._reported:
            if event.candidate_id and event.candidate_id == existing.candidate_id:
                return True
            if event.event_type != existing.event_type:
                continue
            distance = math.hypot(event.x - existing.x, event.y - existing.y)
            if distance <= self.merge_distance:
                return True
        return False
