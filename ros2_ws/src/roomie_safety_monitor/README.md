# Roomie Safety Monitor

안전 모니터 패키지로, 동적 장애물 감지 시 Nav2 네비게이션을 안전하게 제어합니다.

## 기능

- `/vs/obstacle` 토픽에서 장애물 정보 수신
- `/goal_pose` 토픽에서 Nav2 목표 모니터링
- **거리 기반 안전 모니터링** (1미터 이내 동적 장애물 시 정지)
- 동적 장애물 감지 시 Nav2 네비게이션 중단
- 장애물 사라진 후 5초 대기 후 원래 목표로 재시작
- **테스트용 장애물 시뮬레이터** 포함

## 사용법

### 실행

#### 단독 실행
```bash
# Safety Monitor만 실행
ros2 run roomie_safety_monitor safety_monitor_node

# 장애물 시뮬레이터만 실행
ros2 run roomie_safety_monitor obstacle_simulator
```

#### Launch 파일 사용
```bash
# Safety Monitor만 실행
ros2 launch roomie_safety_monitor safety_monitor_launch.py

# Safety Monitor + 장애물 시뮬레이터 (테스트용)
ros2 launch roomie_safety_monitor safety_monitor_test_launch.py
```

### 토픽

#### 구독 토픽
- `/vs/obstacle` (roomie_msgs/Obstacle): 장애물 감지 정보
  - `robot_id`: 장애물 ID (0=장애물 없음)
  - `dynamic`: 동적/정적 장애물 구분
  - `x`, `y`: 장애물 상대 위치
  - `depth`: 장애물까지의 거리 (미터)
- `/goal_pose` (geometry_msgs/PoseStamped): Nav2 목표

#### 발행 토픽 (시뮬레이터)
- `/vs/obstacle` (roomie_msgs/Obstacle): 시뮬레이션된 장애물 정보

#### 액션 클라이언트
- `navigate_to_pose` (nav2_msgs/NavigateToPose): Nav2 제어

## 동작 흐름

1. rviz2에서 Nav2 목표 설정
2. Nav2 네비게이션 시작
3. **1미터 이내 동적 장애물 감지 시 즉시 정지**
4. 장애물 사라짐 감지
5. 5초 대기 후 원래 목표로 재시작

## 테스트

### 장애물 시뮬레이터 동작
- **10초마다 동적 장애물 생성** (3초간 유지)
- 랜덤한 위치에 장애물 배치 (로봇 전방 0.5~1.5m)
- 거리 기반 안전 체크 (1미터 이내 시 정지)

### 로그 확인
```bash
# Safety Monitor 로그
ros2 run roomie_safety_monitor safety_monitor_node

# 장애물 시뮬레이터 로그
ros2 run roomie_safety_monitor obstacle_simulator
```

## 의존성

- rclpy
- geometry_msgs
- nav2_msgs
- roomie_msgs
- tf2_ros

## 설정

### 안전 거리 설정
`safety_monitor_node.py`에서 `self.safety_distance` 값을 수정하여 안전 거리를 조정할 수 있습니다.

```python
self.safety_distance = 1.0  # 1미터 이내 장애물 시 정지
```
