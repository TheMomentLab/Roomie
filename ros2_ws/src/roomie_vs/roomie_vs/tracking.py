import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from std_srvs.srv import Trigger

from roomie_msgs.action import Enroll
from roomie_msgs.msg import Tracking


class PersonRegistry:
    def __init__(self):
        self.is_enrolling = False
        self.enroll_lock = threading.Lock()
        self._stop_flag = False

    def start_enroll(self, duration_sec: float):
        with self.enroll_lock:
            self.is_enrolling = True
            self._stop_flag = False
        # Placeholder: start timer/thread to collect samples for duration

    def cancel_enroll(self):
        with self.enroll_lock:
            self._stop_flag = True
            self.is_enrolling = False


class PersonTracker:
    def __init__(self):
        self.is_tracking = False
        self.target_id: int = 1
        self._lost: bool = True

    def start(self):
        self.is_tracking = True
        self._lost = True

    def stop(self):
        self.is_tracking = False

    def process_frame(self, color_frame) -> Optional[Tracking]:
        if not self.is_tracking:
            return None
        # Placeholder: no model yet. Emit minimal event-only message
        msg = Tracking()
        msg.id = int(self.target_id)
        msg.event = 1  # LOST
        return msg


class TrackingInterface:
    def __init__(self, node: Node):
        self.node = node
        self.registry = PersonRegistry()
        self.tracker = PersonTracker()

        # Publisher
        self.tracking_pub = node.create_publisher(Tracking, '/vs/tracking', 10)

        # Action server for enroll
        self.enroll_action = ActionServer(
            node,
            Enroll,
            '/vs/action/enroll',
            execute_callback=self._execute_enroll,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
        )

        # Stop tracking service
        self.stop_srv = node.create_service(Trigger, '/vs/command/stop_tracking', self._on_stop_tracking)

        # Timer for publishing tracking state (30 Hz tentative)
        self.timer = node.create_timer(1.0/30.0, self._on_timer)

    # Action handlers
    def _on_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self.registry.cancel_enroll()
        return CancelResponse.ACCEPT

    async def _execute_enroll(self, goal_handle):
        duration_sec = float(goal_handle.request.duration_sec)
        if duration_sec <= 0.0:
            duration_sec = 5.0
        self.registry.start_enroll(duration_sec)

        # Minimal feedback loop
        import asyncio
        elapsed = 0.0
        feedback = Enroll.Feedback()
        while elapsed < duration_sec and self.registry.is_enrolling:
            await asyncio.sleep(0.2)
            elapsed += 0.2
            feedback.progress = min(1.0, elapsed / duration_sec)
            goal_handle.publish_feedback(feedback)
        self.registry.cancel_enroll()

        result = Enroll.Result()
        result.success = True
        goal_handle.succeed()
        return result

    # Service handler
    def _on_stop_tracking(self, request, response):
        self.tracker.stop()
        response.success = True
        response.message = ''
        return response

    # Timer loop
    def _on_timer(self):
        try:
            # Get rear camera from node if available
            camera = getattr(self.node, 'current_rear_camera', None)
            if camera is None:
                return
            depth_frame, color_frame = camera.get_frames()
            if color_frame is None:
                return
            msg = self.tracker.process_frame(color_frame)
            if msg is not None:
                self.tracking_pub.publish(msg)
        except Exception as e:
            self.node.get_logger().debug(f"tracking timer error: {e}") 