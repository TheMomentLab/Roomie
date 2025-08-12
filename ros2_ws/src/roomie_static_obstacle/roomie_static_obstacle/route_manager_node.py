#!/usr/bin/env python3
"""
정적 장애물 경로 관리자 노드

기능 요약
- 시작/목표 웨이포인트를 기준으로 그래프 경로(A*)를 계산하여 Nav2로 순차 이동
- /vs/obstacle에서 수신한 정적 장애물을 현재 진행 방향 기준으로 웨이포인트로 분류
- 분류된 웨이포인트가 현재 경로 상(앞쪽)에 있으면 Nav2 목표를 취소하고 경로 재계산
- 시작 시점과 매 웨이포인트 도착 후(방향 전환 시점)에 장애물 재판정

참고
- 경로 탐색 로직은 src/waypoint.py의 A*를 간소화하여 사용
- Nav2 인터페이스는 roomie_safety_monitor/safety_monitor_node.py를 참고
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from roomie_msgs.msg import Obstacle

from .config import (
    WAYPOINTS,
    IN_PATH_X_RANGE,
    classify_forward_waypoint,
    classify_forward_waypoint_relative,
    classify_vertical_waypoint,
)


@dataclass
class AStarWaypoint:
    id: str
    x: float
    y: float
    neighbors: List[str]


class AStar:
    """간단한 A* (격자 그래프용)"""

    def __init__(self, waypoints: Dict[str, Dict]):
        self.nodes: Dict[str, AStarWaypoint] = {
            wp_id: AStarWaypoint(
                id=wp_id,
                x=float(info['x']),
                y=float(info['y']),
                neighbors=list(info['neighbors']),
            )
            for wp_id, info in waypoints.items()
        }

    def heuristic(self, a: str, b: str) -> float:
        na, nb = self.nodes[a], self.nodes[b]
        return abs(na.x - nb.x) + abs(na.y - nb.y)

    def find_path(self, start_id: str, end_id: str, blocked: Set[str]) -> Optional[List[str]]:
        open_set: Set[str] = {start_id}
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {nid: float('inf') for nid in self.nodes}
        f_score: Dict[str, float] = {nid: float('inf') for nid in self.nodes}
        g_score[start_id] = 0.0
        f_score[start_id] = self.heuristic(start_id, end_id)

        while open_set:
            current = min(open_set, key=lambda n: f_score[n])
            if current == end_id:
                # reconstruct
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            open_set.remove(current)
            for nb in self.nodes[current].neighbors:
                if nb in blocked:
                    continue
                tentative = g_score[current] + 1.0
                if tentative < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tentative
                    f_score[nb] = tentative + self.heuristic(nb, end_id)
                    if nb not in open_set:
                        open_set.add(nb)

        return None


class StaticRouteManager(Node):
    """정적 장애물 기반 경로 관리자"""

    def __init__(self) -> None:
        super().__init__('static_route_manager')

        # 파라미터: 시작/목표 웨이포인트 및 경로내 x범위
        self.declare_parameter('start_wp', 'A1')
        self.declare_parameter('goal_wp', 'A5')
        self.declare_parameter('x_range_min', IN_PATH_X_RANGE[0])
        self.declare_parameter('x_range_max', IN_PATH_X_RANGE[1])
        self.declare_parameter('yaw_change_threshold_deg', 5.0)

        self.start_wp: str = self.get_parameter('start_wp').get_parameter_value().string_value
        self.goal_wp: str = self.get_parameter('goal_wp').get_parameter_value().string_value
        self.x_min: float = float(self.get_parameter('x_range_min').get_parameter_value().double_value)
        self.x_max: float = float(self.get_parameter('x_range_max').get_parameter_value().double_value)
        self.yaw_change_threshold_rad: float = math.radians(
            float(self.get_parameter('yaw_change_threshold_deg').get_parameter_value().double_value)
        )

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 장애물 구독
        self.obstacle_sub = self.create_subscription(Obstacle, '/vs/obstacle', self.obstacle_cb, 10)

        # 경로/상태 관리
        self.astar = AStar(WAYPOINTS)
        self.blocked: Set[str] = set()
        self.path: Optional[List[str]] = None
        self.path_index: int = 0
        self.current_goal_handle = None
        self.current_wp: str = self.start_wp
        self.last_obstacle: Optional[Obstacle] = None
        # 관찰 윈도우(출발/방향전환 시 1초 수집)
        self.observation_active: bool = False
        self.observation_msgs: List[Obstacle] = []
        self.observation_timer = None
        self.observation_context: Optional[str] = None
        # 회전-이동 시퀀스 관리
        self.current_goal_kind: Optional[str] = None  # 'rotate' | 'move'
        self.pending_next_wp: Optional[str] = None
        self.pending_next_yaw: Optional[float] = None
        self.last_movement_yaw: Optional[float] = None

        self.get_logger().info(
            f"경로 관리자 시작 - start={self.start_wp}, goal={self.goal_wp}, x in [{self.x_min:.2f}, {self.x_max:.2f}]"
        )

        # 초기 경로 계산 후, 먼저 회전 정렬
        self.recalculate_path()
        self.start_rotate_to_next()

    # ---------- 콜백/핵심 로직 ----------
    def obstacle_cb(self, msg: Obstacle) -> None:
        self.last_obstacle = msg
        # 동적 장애물은 안전 모니터가 처리하므로 이 노드에서는 정적만 고려
        if msg.dynamic:
            return

        # 관찰 윈도우 외에는 정적 장애물 토픽을 사용하지 않음(요청사항)
        if self.observation_active:
            self.observation_msgs.append(msg)
        else:
            return

    def recalculate_path(self) -> None:
        self.path = self.astar.find_path(self.current_wp, self.goal_wp, self.blocked)
        if self.path:
            # 현재 노드가 경로 내 몇 번째인지 동기화
            if self.current_wp in self.path:
                self.path_index = self.path.index(self.current_wp)
            else:
                self.current_wp = self.path[0]
                self.path_index = 0
            path_str = ' -> '.join(self.path)
            self.get_logger().info(f"경로: {path_str}")
        else:
            self.get_logger().error("경로를 찾을 수 없음")

    def advance_if_possible(self) -> None:
        if not self.path or self.path_index >= len(self.path) - 1:
            self.get_logger().info("목표에 도달했거나 유효한 다음 웨이포인트가 없음")
            return

        # 다음 세그먼트로 진행하기 전, 먼저 회전으로 진행방향 정렬부터 수행
        self.start_rotate_to_next()

    def send_nav_goal(self, pose: PoseStamped, kind: str) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 서버 연결 실패')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_cb)
        self.current_goal_kind = kind
        if kind == 'move':
            self.get_logger().info(
                "이동 시작 → (%.2f, %.2f)" % (pose.pose.position.x, pose.pose.position.y)
            )

    def goal_response_cb(self, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('목표 거부됨')
                return
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_cb)
        except Exception as e:
            self.get_logger().error(f'목표 설정 오류: {e}')

    def nav_result_cb(self, future) -> None:
        try:
            result = future.result()
            # SUCCEEDED=4, CANCELED=5, ABORTED=6 (Nav2는 enum이 다를 수 있으므로 상태코드만 확인)
            if getattr(result, 'status', None) == 4:
                if self.current_goal_kind == 'rotate':
                    # 회전 완료 → 정지 상태에서 3초 관찰 후 진행
                    self.start_observation_window(context='aligned', duration_s=3.0)
                else:
                    # 이동 완료 → 현재 웨이포인트 갱신 전, 방금 이동한 세그먼트의 yaw 기록
                    if self.path and self.path_index < len(self.path) - 1:
                        cur_id = self.path[self.path_index]
                        nxt_id = self.path[self.path_index + 1]
                        fx, fy = WAYPOINTS[cur_id]['x'], WAYPOINTS[cur_id]['y']
                        tx, ty = WAYPOINTS[nxt_id]['x'], WAYPOINTS[nxt_id]['y']
                        self.last_movement_yaw = math.atan2(ty - fy, tx - fx)
                        # 현재 웨이포인트 갱신
                        self.current_wp = nxt_id
                        self.path_index += 1
                    # 다음 세그먼트 진행
                    self.advance_if_possible()
            elif getattr(result, 'status', None) == 5:
                self.get_logger().info('네비게이션 취소됨')
            else:
                self.get_logger().warn(f'네비게이션 실패/중단: status={getattr(result, "status", None)}')
        except Exception as e:
            self.get_logger().error(f'결과 처리 오류: {e}')

    # ---------- 보조 로직 ----------
    def make_pose_to_wp(self, from_wp: str, to_wp: str) -> PoseStamped:
        fx, fy = WAYPOINTS[from_wp]['x'], WAYPOINTS[from_wp]['y']
        tx, ty = WAYPOINTS[to_wp]['x'], WAYPOINTS[to_wp]['y']
        yaw = math.atan2(ty - fy, tx - fx)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(tx)
        pose.pose.position.y = float(ty)
        pose.pose.position.z = 0.0
        q = self.yaw_to_quaternion(yaw)
        pose.pose.orientation = q
        return pose

    def make_pose_at_wp_with_yaw(self, wp_id: str, yaw: float) -> PoseStamped:
        x, y = WAYPOINTS[wp_id]['x'], WAYPOINTS[wp_id]['y']
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        half = yaw / 2.0
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half)
        q.w = math.cos(half)
        return q

    def cancel_navigation(self) -> None:
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def is_wp_ahead_in_path(self, wp_id: str) -> bool:
        if not self.path:
            return False
        try:
            idx = self.path.index(wp_id)
            return idx > self.path_index
        except ValueError:
            return False

    def check_and_replan_for_static_obstacle(self, context: str) -> None:
        """현재/다음 세그먼트 진행 방향 기준으로 마지막 장애물을 평가하여 필요시 재계산"""
        if not self.path or self.path_index >= len(self.path) - 1:
            return
        if self.last_obstacle is None:
            return
        if bool(self.last_obstacle.dynamic):
            return

        blocked_wp = self.classify_obstacle_to_waypoint(self.last_obstacle)
        if blocked_wp and self.is_wp_ahead_in_path(blocked_wp):
            self.get_logger().warn(f"[{context}] 정적 장애물로 {blocked_wp} 차단 → 경로 재계산")
            self.blocked.add(blocked_wp)
            self.cancel_navigation()
            self.recalculate_path()

    def classify_obstacle_to_waypoint(self, obs: Obstacle) -> Optional[str]:
        """x 정규화 범위 내일 때만 깊이→웨이포인트 분류 수행"""
        x_norm = float(getattr(obs, 'x', 0.5))
        depth = float(getattr(obs, 'depth', 0.0))
        if not (self.x_min <= x_norm <= self.x_max):
            return None

        # 현재 세그먼트 방향 판단
        if not self.path or self.path_index >= len(self.path) - 1:
            return None
        cur_id = self.path[self.path_index]
        nxt_id = self.path[self.path_index + 1]

        cur_row, cur_col = cur_id[0], cur_id[1]
        nxt_row, nxt_col = nxt_id[0], nxt_id[1]

        if cur_row == nxt_row:
            # 수평 진행 (행 고정, 열 변경)
            heading_row = cur_row
            if heading_row in ('A', 'C'):
                # 상대 진행 방향(A5→A4 등) 고려하여 깊이→열 오프셋 계산
                rel_wp = classify_forward_waypoint_relative(depth, cur_id, nxt_id)
                return rel_wp
            else:
                return None  # B 행은 사용 안함
        else:
            # 수직 진행 (열 고정, 행 변경)
            column = cur_col  # 같은 열이어야 함
            # 원점: A→B/ C 방향이면 A, C→B/ A 방향이면 C
            origin = 'A' if cur_row == 'A' else 'C'
            return classify_vertical_waypoint(depth, column, origin)

    # ---------- 관찰 윈도우 / 시퀀스 ----------
    def start_observation_window(self, context: str, duration_s: float = 1.0) -> None:
        """관찰 윈도우 시작: 장애물 버퍼링 후 일괄 분류/재계산"""
        if self.observation_active:
            return
        self.observation_active = True
        self.observation_msgs = []
        self.observation_context = context
        # 타이머 시작
        self.observation_timer = self.create_timer(duration_s, self.finish_observation_window)
        self.get_logger().info(f"[{context}] {duration_s:.1f}초 관찰 시작")

    def finish_observation_window(self) -> None:
        # 타이머 단발성 처리
        if self.observation_timer:
            self.observation_timer.cancel()
            self.observation_timer = None
        context = self.observation_context or 'unknown'
        self.observation_context = None

        # 버퍼 분류 → 차단 웨이포인트 집합 구성
        blocked_new: Set[str] = set()
        classified_logs: List[str] = []
        for obs in self.observation_msgs:
            wp = self.classify_obstacle_to_waypoint(obs)
            if wp and self.is_wp_ahead_in_path(wp):
                blocked_new.add(wp)
                classified_logs.append(f"wp={wp}, x={getattr(obs,'x',0):.2f}, d={getattr(obs,'depth',0):.2f}")

        self.get_logger().info(
            f"[{context}] 관찰 완료: 수신 {len(self.observation_msgs)}건, 차단 {sorted(list(blocked_new))}"
        )
        for line in classified_logs[:10]:  # 로그 과다 방지
            self.get_logger().info(f"[{context}] 분류: {line}")

        # 상태 리셋
        self.observation_msgs = []
        self.observation_active = False

        # 경로 재계산 (필요 시)
        if blocked_new:
            self.blocked.update(blocked_new)
            self.cancel_navigation()
            self.recalculate_path()

        # 관찰 후 진행: 이미 회전 완료 상태이므로 즉시 이동
        self.advance_after_observation()

    def advance_after_observation(self) -> None:
        if not self.path or self.path_index >= len(self.path) - 1:
            self.get_logger().info("목표에 도달했거나 유효한 다음 웨이포인트가 없음")
            return
        # 다음 세그먼트 타깃/요 회전 각도 계산
        next_wp = self.path[self.path_index + 1]
        fx, fy = WAYPOINTS[self.current_wp]['x'], WAYPOINTS[self.current_wp]['y']
        tx, ty = WAYPOINTS[next_wp]['x'], WAYPOINTS[next_wp]['y']
        desired_yaw = math.atan2(ty - fy, tx - fx)
        # 관찰이 끝났으므로 이동 목표 전송
        move_pose = self.make_pose_to_wp(self.current_wp, next_wp)
        self.send_nav_goal(move_pose, kind='move')

    def start_rotate_to_next(self) -> None:
        """다음 세그먼트 진행전, 제자리에서 목표 진행각도로 회전"""
        if not self.path or self.path_index >= len(self.path) - 1:
            return
        next_wp = self.path[self.path_index + 1]
        fx, fy = WAYPOINTS[self.current_wp]['x'], WAYPOINTS[self.current_wp]['y']
        tx, ty = WAYPOINTS[next_wp]['x'], WAYPOINTS[next_wp]['y']
        desired_yaw = math.atan2(ty - fy, tx - fx)
        # 회전 필요 여부 판단: 직전 이동 yaw와 비교(초기엔 반드시 회전)
        need_rotate = False
        if self.last_movement_yaw is None:
            need_rotate = True
        else:
            diff = self.normalize_angle(desired_yaw - self.last_movement_yaw)
            need_rotate = abs(diff) >= self.yaw_change_threshold_rad

        if need_rotate:
            self.pending_next_wp = next_wp
            self.pending_next_yaw = desired_yaw
            rotate_pose = self.make_pose_at_wp_with_yaw(self.current_wp, desired_yaw)
            # 회전 로그는 한 줄만 출력
            self.get_logger().info(
                f"회전 시작(Δyaw={abs(self.normalize_angle(desired_yaw - (self.last_movement_yaw or 0.0)))*180.0/math.pi:.1f}°)"
            )
            self.send_nav_goal(rotate_pose, kind='rotate')
        else:
            # 회전 생략 → 바로 이동 목표 전송
            self.get_logger().info(
                f"회전 생략(Δyaw={abs(self.normalize_angle(desired_yaw - self.last_movement_yaw))*180.0/math.pi:.1f}°) → 이동"
            )
            move_pose = self.make_pose_to_wp(self.current_wp, next_wp)
            self.send_nav_goal(move_pose, kind='move')

    def normalize_angle(self, a: float) -> float:
        """[-pi, pi] 범위로 정규화"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticRouteManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


