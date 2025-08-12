# Roomie GC (Guide/Companion)

이 패키지는 길안내 시나리오를 담당하는 GC 노드를 제공합니다.

## 상태(State)
- WAITING_DEST_INPUT: 목적지 입력 대기
- ENROLL_REQUESTED: 등록 모드 전환 요청
- ENROLLING: 등록 진행
- WAITING_TRACKING_REQUEST: 등록 완료 후 추적모드 전환 요청 대기
- START_MOVING: 추적모드 전환 및 주행 시작
- NAVIGATING: 길안내 주행 중
- ARRIVED: 목적지 도착 처리
- STOP_TRACKING: 추적 중지 요청
- RETURNING: 대기 위치로 복귀

## 주요 토픽/서비스/액션
- Topic(pub): `/robot_gui/event`, `/roomie/status/robot_state`, `/ioc/read_card_request`
- Topic(sub): `/ioc/read_card_response`, `/vs/tracking`
- Service(cli): `/vs/command/set_vs_mode`, `/vs/command/stop_tracking`
- Action(cli): `/vs/action/enroll`, `navigate_to_pose`

## 시나리오 개요
1. 목적지 입력 대기
   - RGUI 102 수신 → 상태 정렬
   - RGUI 103 수신 → 카드 인식 요청
   - 카드 인식 성공 → RGUI 9 발행
   - RGUI 106 수신 → 등록 모드 전환

2. 등록
   - VS 모드 전환(mode=1) → 등록 액션 → 성공 시 RGUI 10
   - 등록 중 106 수신 시 보류 후 등록 완료 즉시 START_MOVING 전이
   - 등록 완료 후 106 수신 시 START_MOVING 전이

3. 추적/주행 시작
   - VS 모드 전환(mode=2), 로봇 상태 21→22, Nav2 목표 전송
   - `/vs/tracking` 이벤트 처리(START_MOVING/NAVIGATING에서만 유효)
     - 1=SLOW_DOWN: 감속 속도 적용
     - 2=LOST: 거의 정지 + RGUI 21
     - 3=RESUME: 정상 복귀 + RGUI 22
     - 0=NONE: 정상 복귀
   - Nav2 결과 수신 → ARRIVED

4. 도착/복귀
   - VS 추적 중지 → 성공 후 RGUI 11
   - VS 모드 전환(mode=0) → 로봇 상태 22→30
   - 대기 위치(0) 복귀 주행 → 도착 시 로봇 상태 30→2

## 속도 제한 정책 (Nav2 DWB)
- 기본: 절대값 모드(`sl_percentage=false`)
  - SLOW_DOWN: `sl_slow_scale` [m/s]
  - LOST: `sl_lost_stop_speed` [m/s]
  - RESUME/NONE: `sl_resume_scale=0.0` → 리미터 해제
- 짧게 N회 재발행(`sl_repeat_count`, `sl_repeat_period`)으로 반영 보장

## 런치 파라미터 예시
```bash
ros2 launch roomie_gc roomie_gc.launch.py \
  sl_slow_scale:=0.10 sl_lost_stop_speed:=0.0005 \
  sl_repeat_count:=8 sl_repeat_period:=0.1
```

