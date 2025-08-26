import threading
import time
from rclpy.logging import Logger

try:
    import pynvml
    PYNVML_AVAILABLE = True
except ImportError:
    PYNVML_AVAILABLE = False

class GPUResourceMonitor:
    """NVIDIA GPU의 메모리 사용량을 모니터링하고, 설정된 임계값을 초과하면 경고를 로깅하는 클래스입니다."""

    def __init__(self, logger: Logger, max_memory_mb: int = 4000):
        """
        모니터 초기화

        Args:
            logger (Logger): ROS2 로거 객체
            max_memory_mb (int): 메모리 사용량 경고를 위한 임계값 (MB 단위)
        """
        self.logger = logger
        self.max_memory_mb = max_memory_mb
        self.is_monitoring = False
        self.monitoring_thread = None

        if not PYNVML_AVAILABLE:
            self.logger.warning("pynvml 라이브러리를 찾을 수 없습니다. GPU 모니터링이 비활성화됩니다.")
            self.logger.warning("설치 명령어: pip install pynvml")
            return

        try:
            pynvml.nvmlInit()
            self.handle = pynvml.nvmlDeviceGetHandleByIndex(0)  # 0번 GPU를 대상으로 함
            self.logger.info("GPU 모니터가 성공적으로 초기화되었습니다.")
        except pynvml.NVMLError as e:
            self.logger.error(f"pynvml 초기화에 실패했습니다: {e}. GPU 모니터링을 사용할 수 없습니다.")
            self.handle = None

    def start_monitoring(self, interval_sec: int = 5):
        """지정된 간격으로 GPU 메모리를 확인하는 백그라운드 스레드를 시작합니다."""
        if not PYNVML_AVAILABLE or self.handle is None:
            return

        if not self.is_monitoring:
            self.is_monitoring = True
            self.monitoring_thread = threading.Thread(target=self._monitor_loop, args=(interval_sec,), daemon=True)
            self.monitoring_thread.start()
            self.logger.info(f"{interval_sec}초 간격으로 GPU 모니터링을 시작합니다.")

    def stop_monitoring(self):
        """모니터링 스레드를 중지합니다."""
        if self.is_monitoring:
            self.is_monitoring = False
            if self.monitoring_thread:
                self.monitoring_thread.join()
            self.logger.info("GPU 모니터링이 중지되었습니다.")
        
        if PYNVML_AVAILABLE:
            try:
                pynvml.nvmlShutdown()
            except pynvml.NVMLError:
                pass # 이미 종료되었을 수 있음

    def _monitor_loop(self, interval_sec: int):
        """주기적으로 GPU 메모리 사용량을 확인하고 임계값 초과 시 경고를 출력하는 내부 루프입니다."""
        while self.is_monitoring:
            try:
                mem_info = pynvml.nvmlDeviceGetMemoryInfo(self.handle)
                used_mb = mem_info.used / (1024**2)
                total_mb = mem_info.total / (1024**2)

                self.logger.debug(f"GPU 메모리 사용량: {used_mb:.2f}MB / {total_mb:.2f}MB")

                if used_mb > self.max_memory_mb:
                    self.logger.warning(
                        f"GPU 메모리 사용량({used_mb:.2f}MB)이 임계값({self.max_memory_mb}MB)을 초과했습니다!"
                    )
            except pynvml.NVMLError as e:
                self.logger.error(f"GPU 정보 조회 중 오류 발생: {e}")
                # 오류 발생 시 루프를 중단하여 반복적인 에러 로깅을 방지
                self.is_monitoring = False
                break
            
            time.sleep(interval_sec)