from __future__ import annotations

import uuid

from mission_control_interfaces.action import ReportEvent
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class ReporterStub(Node):
    """Test-only reporter that acknowledges an event without producing audio."""

    def __init__(self) -> None:
        super().__init__("event_reporter_stub")
        action_name = self.declare_parameter(
            "action_name", "/event_reporter/report"
        ).value
        self._server = ActionServer(self, ReportEvent, action_name, self._execute)

    def _execute(self, goal_handle):
        feedback = ReportEvent.Feedback()
        feedback.phase = ReportEvent.Feedback.PLAYING
        feedback.status = "test reporter accepted event"
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()
        result = ReportEvent.Result()
        result.success = True
        result.report_id = str(uuid.uuid4())
        result.message = "test report completed"
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReporterStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
