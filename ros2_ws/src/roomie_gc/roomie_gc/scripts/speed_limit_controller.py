#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from roomie_msgs.msg import Tracking
from nav2_msgs.msg import SpeedLimit


class SpeedLimitController(Node):
    def __init__(self):
        super().__init__('speed_limit_controller')
        # Parameters
        # Match Nav2 DWB default without leading slash
        self.declare_parameter('speed_limit_topic', 'speed_limit')
        # Absolute m/s mode by default (more predictable on DWB)
        self.declare_parameter('slow_scale', 0.2)          # m/s when percentage=False
        self.declare_parameter('resume_scale', 0.0)        # 0.0 disables limiter on DWB
        self.declare_parameter('percentage', False)
        self.declare_parameter('lost_stop_speed', 0.001)   # nearly stop in absolute mode
        self.declare_parameter('repeat_count', 5)
        self.declare_parameter('repeat_period', 0.1)

        speed_limit_topic = self.get_parameter('speed_limit_topic').value
        self.slow_scale = float(self.get_parameter('slow_scale').value)
        self.resume_scale = float(self.get_parameter('resume_scale').value)
        self.percentage = bool(self.get_parameter('percentage').value)
        self.repeat_count_default = int(self.get_parameter('repeat_count').value)
        self.repeat_period = float(self.get_parameter('repeat_period').value)
        self.lost_stop_speed = float(self.get_parameter('lost_stop_speed').value)

        # Pub/Sub
        self.pub = self.create_publisher(SpeedLimit, speed_limit_topic, QoSProfile(depth=10))
        self.sub = self.create_subscription(Tracking, '/vs/tracking', self.on_tracking, QoSProfile(depth=10))
        # Re-publish helper
        self._repeat_remaining = 0
        self._last_msg = None
        self.create_timer(self.repeat_period, self._repeat_publish)

        self.get_logger().info(
            f'SpeedLimitController 시작: publish to {speed_limit_topic} '
            f'(slow={self.slow_scale}, resume={self.resume_scale})'
        )

    def on_tracking(self, msg: Tracking):
        # event: 0=NONE, 1=SLOW_DOWN, 2=LOST, 3=RESUME
        sl = SpeedLimit()
        sl.percentage = self.percentage
        if msg.event == 1:  # slow_down
            sl.speed_limit = float(self.slow_scale)
            self._publish_with_repeat(sl, tag='SLOW_DOWN')
        elif msg.event == 2:  # lost → stop
            # In absolute mode, use near-zero to effectively stop (0.0 would disable on DWB)
            sl.speed_limit = float(self.lost_stop_speed)
            self._publish_with_repeat(sl, tag='LOST')
        elif msg.event == 3:  # resume
            # 0.0 with percentage=False disables speed limiter on DWB → full speed
            sl.speed_limit = float(self.resume_scale)
            self._publish_with_repeat(sl, tag='RESUME')
        elif msg.event == 0:  # NONE → normal speed
            sl.speed_limit = float(self.resume_scale)
            self._publish_with_repeat(sl, tag='NONE→NORMAL')

    def _publish_with_repeat(self, msg: SpeedLimit, tag: str = ''):
        self.pub.publish(msg)
        self._last_msg = msg
        self._repeat_remaining = self.repeat_count_default
        self.get_logger().info(
            f'set speed_limit={msg.speed_limit:.2f} (percentage={msg.percentage}){" ["+tag+"]" if tag else ""}'
        )

    def _repeat_publish(self):
        if self._repeat_remaining > 0 and self._last_msg is not None:
            self.pub.publish(self._last_msg)
            self._repeat_remaining -= 1


def main():
    rclpy.init()
    node = SpeedLimitController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


