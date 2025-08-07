# Roomie RC (Robot Control) Module

로봇 제어 Python 모듈 - 서비스 로봇의 중심 제어 노드

## 📁 구조

- `rc_node.py` - 메인 로봇 제어 노드 (중심 허브)
- `rms_client.py` - RMS(Roomie Main Service) 통신 클라이언트
- `gui_client.py` - RGUI(Robot GUI) 통신 클라이언트  
- `vs_client.py` - VS(Vision Service) 통신 클라이언트
- `ioc_client.py` - IOC(IO Controller) 통신 클라이언트

## 🚀 주요 기능

### ✅ 구현 완료
- **RMS 통신**: 작업 할당, 상태 발행, 이벤트 처리
- **GUI 통신**: 서비스/토픽 양방향 통신, 사용자 인터랙션
- **하드웨어 통신**: VS(위치 감지), IOC(서랍/센서 제어)  
- **배송 프로세스**: 픽업/배송 스켈레톤 구현
- **상태 관리**: 로봇 상태 자동 전환

### 🔄 TODO
- 실제 내비게이션 연동
- 배송 작업 알고리즘 고도화
- 센서 실시간 모니터링

## 📊 상태 관리 시스템

### 로봇 상태 (Robot State)
로봇의 작업 할당 가능 여부를 나타내는 상태입니다. **로봇에서만 수정 가능**합니다.

- **0**: 작업 불가능
- **1**: 작업 가능
- **2**: 작업 입력 중
- **3**: 작업 수행 중
- **4**: 복귀 대기 중
- **5**: 복귀 중
- **6**: 작업 실패
- **7**: 시스템 오류

### 작업 상태 (Task State)
할당받은 작업의 진행 과정을 나타내는 상태입니다. **메인 서버에서만 생성/수정 가능**합니다.

- **0**: 접수됨         # 배송
- **1**: 준비 완료
- **2**: 로봇 할당됨
- **3**: 픽업 장소로 이동
- **4**: 픽업 대기 중
- **5**: 배송 중
- **6**: 픽업 도착
- **7**: 수령 완료
- **10**: 호출 이동 중  # 호출
- **11**: 호출 도착
- **20**: 길안내 중     # 길안내
- **21**: 길안내 도착

## 📡 통신 인터페이스

### RMS 통신
- **Action**: `/roomie/action/perform_task` (작업 할당 받기)
- **Publisher**: 로봇 상태, 배터리, 위치, 이벤트 발행
- **Subscriber**: 작업 상태 구독

### GUI 통신  
- **Topic**: `/robot_gui/event` (양방향 이벤트)
- **Service**: 카운트다운, 도어 잠금 해제

### 하드웨어 통신
- **VS**: 위치 감지 (`/vs/command/location`)
- **IOC**: 서랍 제어, 센서 확인

## 🏃 실행 방법

```bash
# 빌드
colcon build --packages-select roomie_rc

# 실행  
source install/setup.bash
ros2 run roomie_rc rc_node
``` 