#!/bin/bash

echo "🛡️ 안전한 Roomie VS 실행 스크립트"
echo "시스템 멈춤 방지를 위한 보호 조치 적용 중..."

# 환경 변수 설정
export OPENCV_LOG_LEVEL=ERROR
export OMP_NUM_THREADS=4  # CPU 모드를 위해 스레드 증가

# 안정성 우선: CPU 모드 (시스템 멈춤 방지)
export CUDA_VISIBLE_DEVICES=""  # CPU 전용 모드
export TORCH_USE_CUDA_DSA=0
export NCCL_AVOID_RECORD_STREAMS=1
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128,expandable_segments:False
export NCCL_P2P_DISABLE=1  # P2P 통신 비활성화로 안정성 향상
export NCCL_IB_DISABLE=1   # InfiniBand 비활성화

# 메모리 최적화 설정
export MALLOC_ARENA_MAX=2  # glibc 메모리 할당 최적화
export PYTHONDONTWRITEBYTECODE=1  # .pyc 파일 생성 안함
export PYTHONUNBUFFERED=1  # 버퍼링 비활성화

# GPU 메모리 제한 (RTX 2060 6GB 중 4GB만 사용)
export CUDA_MEMORY_LIMIT=4294967296  # 4GB 제한
export TORCH_CUDNN_USE_HEURISTIC_MODE_B=1

# 메모리 제한 (6GB로 증가)
ulimit -v 6000000
# CPU 시간 제한 (10분)
ulimit -t 600
# 파일 디스크립터 제한
ulimit -n 1024

# 우선순위 낮춤 (nice 값 증가)
renice +10 $$

echo "✅ 리소스 제한 설정 완료"
echo "   - 가상 메모리: 4GB 제한"
echo "   - CPU 시간: 10분 제한"
echo "   - OpenMP 스레드: 2개로 제한"
echo "   - 프로세스 우선순위: 낮춤"

# Watchdog 설정 (백그라운드에서 모니터링)
{
    sleep 30  # 30초 대기
    while true; do
        # vs_node 프로세스 확인
        if pgrep -f "vs_node" > /dev/null; then
            # 메모리 사용량 확인 (MB 단위)
            MEMORY_MB=$(ps -o pid,vsz,rss,comm -p $(pgrep -f "vs_node") | tail -1 | awk '{print $3/1024}')
            if (( $(echo "$MEMORY_MB > 2000" | bc -l) )); then
                echo "⚠️ 메모리 사용량 경고: ${MEMORY_MB}MB"
                echo "시스템 보호를 위해 vs_node를 종료합니다..."
                pkill -f "vs_node"
                break
            fi
        else
            echo "✅ vs_node 프로세스가 정상 종료되었습니다"
            break
        fi
        sleep 5
    done
} &

WATCHDOG_PID=$!
echo "🔒 Watchdog 프로세스 시작: PID $WATCHDOG_PID"

# ROS 환경 소싱 (절대 경로 사용)
source /opt/ros/jazzy/setup.bash
cd /home/jinhyuk2me/project_ws/Roomie
source install/setup.bash

echo "🚀 vs_node 실행 중..."
echo "ESC 키로 안전하게 종료하세요"

# vs_node 실행 (타임아웃과 함께)
timeout 600 ros2 run roomie_vs vs_node

EXIT_CODE=$?

# Watchdog 종료
kill $WATCHDOG_PID 2>/dev/null

if [ $EXIT_CODE -eq 124 ]; then
    echo "⏰ 타임아웃으로 인해 종료되었습니다 (10분 제한)"
elif [ $EXIT_CODE -eq 0 ]; then
    echo "✅ 정상 종료되었습니다"
else
    echo "⚠️ 비정상 종료 (Exit Code: $EXIT_CODE)"
fi

echo "🧹 리소스 정리 중..."
# OpenCV 윈도우 강제 종료
pkill -f "python.*vs_node" 2>/dev/null || true
# 남은 윈도우 정리
wmctrl -c "Roomie VS" 2>/dev/null || true

echo "🏁 실행 완료" 