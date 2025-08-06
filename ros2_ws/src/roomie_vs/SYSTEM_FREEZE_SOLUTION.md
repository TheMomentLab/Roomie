# 🚨 Roomie VS 시스템 멈춤 문제 해결 가이드

## 문제 상황
`roomie_vs` 실행 중 컴퓨터 전체가 멈춰서 강제 재시작을 해야 하는 상황

## 🔍 원인 분석
1. **OpenNI2 카메라 드라이버 블로킹**: 카메라 스트림에서 무한 대기
2. **메모리 과부하**: YOLO 모델과 이미지 처리로 인한 메모리 누수
3. **GPU 리소스 충돌**: OpenCV + CUDA + YOLO 동시 사용
4. **GUI 시스템 블로킹**: X11 윈도우 시스템과의 상호작용 문제

---

## 🛡️ 즉시 적용할 해결책

### 1. 안전한 실행 스크립트 사용

**기존 실행 방법 (위험):**
```bash
ros2 run roomie_vs vs_node
```

**새로운 안전한 실행 방법:**
```bash
# 터미널 1: 안전한 실행
cd /home/jinhyuk2me/project_ws/Roomie
./ros2_ws/src/roomie_vs/scripts/safe_vs_runner.sh

# 터미널 2: 실시간 모니터링 (별도 터미널)
cd /home/jinhyuk2me/project_ws/Roomie
python3 ros2_ws/src/roomie_vs/scripts/vs_monitor.py
```

### 2. 보호 기능
- ⏰ **타임아웃**: 10분 후 자동 종료
- 🧠 **메모리 제한**: 4GB 가상 메모리 제한
- 👁️ **실시간 모니터링**: 메모리/CPU 사용량 추적
- 🚨 **자동 종료**: 위험 수준 도달 시 자동 종료
- 🧹 **리소스 정리**: 종료 시 GUI 윈도우 자동 정리

---

## 🔧 시스템 설정 개선

### 1. 스왑 메모리 늘리기 (메모리 부족 방지)
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 2. GPU 메모리 제한 설정
```bash
echo 'export CUDA_VISIBLE_DEVICES=0' >> ~/.bashrc
echo 'export TF_FORCE_GPU_ALLOW_GROWTH=true' >> ~/.bashrc
```

### 3. OpenCV 로그 레벨 조정
```bash
echo 'export OPENCV_LOG_LEVEL=ERROR' >> ~/.bashrc
```

---

## 📊 실시간 모니터링 도구 사용법

### 모니터링 스크립트 실행
```bash
python3 ros2_ws/src/roomie_vs/scripts/vs_monitor.py
```

### 출력 예시
```
🛡️ Roomie VS 노드 모니터링 시작
[2024-01-27 14:30:15] ℹ️ 🔍 VS 노드 모니터링 시작
[2024-01-27 14:30:15] ℹ️ 설정: 메모리 제한 2500MB, CPU 제한 80%
[2024-01-27 14:30:16] ℹ️ PID:12345 | 메모리:1250.3MB | CPU:45.2% | GPU:2048MB/8192MB (25.0%)
[2024-01-27 14:30:19] ⚠️ PID:12345 | 메모리:2700.5MB | CPU:85.1% | 경고: 메모리 과사용, CPU 과사용 | 연속: 1/5
```

---

## 🚨 비상 상황 대처법

### 1. 시스템이 느려질 때
```bash
# 다른 터미널에서
pkill -f "vs_node"
pkill -f "python.*vs_node"
```

### 2. 완전히 멈춘 경우
- `Ctrl + Alt + F2` (TTY2로 전환)
- 로그인 후:
```bash
sudo pkill -9 -f vs_node
sudo pkill -9 python3 
sudo systemctl restart gdm  # GUI 재시작
```

### 3. SSH 접근 가능한 경우
```bash
ssh user@localhost
sudo reboot  # 안전한 재시작
```

---

## ⚙️ 성능 최적화 설정

### 1. 시스템 우선순위 조정
```bash
# vs_node 실행 전
echo 'echo -17 > /proc/self/oom_adj' | sudo tee /etc/sysctl.d/99-roomie-vs.conf
```

### 2. CPU 거버너 설정
```bash
# 성능 모드로 설정
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### 3. 네트워크 최적화
```bash
# ROS2 메시지 버퍼 크기 조정
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp' >> ~/.bashrc
echo 'export CYCLONEDX_URI=cyclonedx.ini' >> ~/.bashrc
```

---

## 📝 문제 발생 시 로그 수집

### 1. 시스템 로그
```bash
# 커널 메시지
dmesg | tail -50

# 시스템 로그
journalctl --since "10 minutes ago" | grep -i error

# GPU 상태
nvidia-smi
```

### 2. 프로세스 정보
```bash
# 메모리 사용량 상위 프로세스
ps aux --sort=-%mem | head -10

# CPU 사용량 상위 프로세스  
ps aux --sort=-%cpu | head -10
```

---

## 🎯 권장 실행 순서

1. **준비 단계**
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ulimit -v 4000000  # 메모리 제한
   ```

2. **모니터링 시작** (터미널 1)
   ```bash
   python3 ros2_ws/src/roomie_vs/scripts/vs_monitor.py
   ```

3. **안전한 실행** (터미널 2)
   ```bash
   ./ros2_ws/src/roomie_vs/scripts/safe_vs_runner.sh
   ```

4. **정상 종료**
   - vs_node GUI에서 `ESC` 키 입력
   - 또는 `Ctrl+C`로 스크립트 종료

---

## 🔄 추가 개선 계획

- [ ] 카메라 연결 상태 자동 감지 및 재연결
- [ ] YOLO 모델 동적 로딩/언로딩
- [ ] GUI 없는 헤드리스 모드
- [ ] 분산 처리 (카메라/AI 분리)
- [ ] 웹 기반 모니터링 대시보드

---

**⚠️ 중요: 이 가이드의 스크립트들을 사용하면 시스템 멈춤 문제를 크게 줄일 수 있습니다!** 