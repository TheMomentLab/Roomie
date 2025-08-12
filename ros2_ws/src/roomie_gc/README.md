# Roomie GC (Guidance Controller)

- start_state로 중간 시작 가능: `waiting_dest_input`, `enroll_requested`, `enrolling`, `start_moving`, `navigating`, `arrived`, `stop_tracking`, `returning`
- debug_mode=true일 때 목적지는 101로 고정
- 5초마다 `/roomie/status/robot_state` 발행
- 인터페이스 확인 후 시작: VS(SetVSMode/StopTracking), VS Enroll action, IOC ReadCardInfo action, RGUI StartCountdown action, Nav2 navigate_to_pose action

## Launch
```bash
ros2 launch roomie_gc roomie_gc.launch.py debug_mode:=true start_state:=waiting_dest_input
```



