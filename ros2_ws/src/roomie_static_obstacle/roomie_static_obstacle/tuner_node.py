#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
try:
    from roomie_msgs.msg import Obstacle
except Exception:  # 빌드시 커스텀 메시지가 없을 경우를 대비한 폴백
    from dataclasses import dataclass
    @dataclass
    class Obstacle:  # type: ignore
        depth: float
        x: float
        dynamic: bool

from .config import (
    IN_PATH_X_RANGE,
    classify_forward_waypoint,
    classify_vertical_waypoint,
)


class StaticObstacleTuner(Node):
    """무구동 튜너 노드
    - /vs/obstacle 구독
    - x 정규화 좌표 기반 경로내 판정
    - 깊이 → 웨이포인트 분류(전방/세로 모드)
    - 튜닝 로그 및 결과 토픽으로 출력
    """

    def __init__(self):
        super().__init__('static_obstacle_tuner')
        # 전방/세로 분류 모드 및 파라미터
        self.declare_parameter('mode', 'forward')  # 'forward' | 'vertical'
        self.declare_parameter('heading_row', 'A')  # 'A'|'B'|'C'
        self.declare_parameter('column', '1')       # '1'|'3'|'5' 등 (vertical 모드)
        self.declare_parameter('vertical_origin', 'A')  # 'A'|'C'
        self.declare_parameter('x_range_min', IN_PATH_X_RANGE[0])
        self.declare_parameter('x_range_max', IN_PATH_X_RANGE[1])

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.heading_row = self.get_parameter('heading_row').get_parameter_value().string_value
        self.column = self.get_parameter('column').get_parameter_value().string_value
        self.vertical_origin = self.get_parameter('vertical_origin').get_parameter_value().string_value
        self.x_min = float(self.get_parameter('x_range_min').get_parameter_value().double_value)
        self.x_max = float(self.get_parameter('x_range_max').get_parameter_value().double_value)

        self.obstacle_sub = self.create_subscription(Obstacle, '/vs/obstacle', self.obstacle_cb, 10)
        self.result_pub = self.create_publisher(String, '/static_obstacle_tuner/result', 10)

        self.get_logger().info(
            f"튜너 시작 - mode={self.mode}, heading_row={self.heading_row}, column={self.column}, vertical_origin={self.vertical_origin}, x in [{self.x_min:.2f}, {self.x_max:.2f}]"
        )

    def obstacle_cb(self, msg: Obstacle):
        x_norm = float(getattr(msg, 'x', 0.5))
        depth = float(getattr(msg, 'depth', 0.0))
        dynamic = bool(getattr(msg, 'dynamic', False))

        in_path = (self.x_min <= x_norm <= self.x_max)

        if self.mode == 'forward':
            waypoint = classify_forward_waypoint(depth, self.heading_row) if in_path else None
        else:  # vertical
            waypoint = classify_vertical_waypoint(depth, self.column, self.vertical_origin) if in_path else None

        status = '경로내' if in_path else '경로외'
        dyn = '동적' if dynamic else '정적'
        wp_str = waypoint if waypoint else '-'
        text = f'{status} {dyn} x={x_norm:.2f} depth={depth:.2f} -> wp={wp_str}'

        self.get_logger().info(text)
        self.result_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = StaticObstacleTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


