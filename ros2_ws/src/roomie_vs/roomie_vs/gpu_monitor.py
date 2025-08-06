#!/usr/bin/env python3

import os
import sys
import time
import threading
from typing import Optional, Callable

try:
    import pynvml
    PYNVML_AVAILABLE = True
except ImportError:
    PYNVML_AVAILABLE = False

class GPUResourceMonitor:
    """GPU 리소스 모니터링 및 자동 제어 클래스"""
    
    def __init__(self, logger, max_memory_mb: int = 4096, check_interval: float = 5.0):
        self.logger = logger
        self.max_memory_mb = max_memory_mb  # GPU 메모리 제한 (MB)
        self.check_interval = check_interval  # 모니터링 주기 (초)
        
        # GPU 모니터링 상태
        self.monitoring = False
        self.monitor_thread = None
        self.gpu_available = False
        self.handle = None
        
        # 콜백 함수들
        self.on_memory_exceeded: Optional[Callable] = None  # 메모리 초과 시 호출
        self.on_gpu_error: Optional[Callable] = None  # GPU 오류 시 호출
        
        # 통계
        self.violation_count = 0
        self.max_memory_used = 0
        
        self._initialize_gpu()
    
    def _initialize_gpu(self):
        """GPU 모니터링 초기화"""
        if not PYNVML_AVAILABLE:
            self.logger.warning("❌ pynvml 모듈을 찾을 수 없습니다. GPU 모니터링 비활성화")
            return
        
        try:
            pynvml.nvmlInit()
            self.handle = pynvml.nvmlDeviceGetHandleByIndex(0)  # 첫 번째 GPU
            
            # GPU 정보 가져오기
            raw_name = pynvml.nvmlDeviceGetName(self.handle)
            # 최신 pynvml에서는 이미 string으로 반환될 수 있음
            if isinstance(raw_name, bytes):
                name = raw_name.decode('utf-8')
            else:
                name = str(raw_name)
            memory_info = pynvml.nvmlDeviceGetMemoryInfo(self.handle)
            total_memory_mb = memory_info.total // (1024 * 1024)
            
            self.gpu_available = True
            self.logger.info(f"🎮 GPU 모니터링 활성화: {name}")
            self.logger.info(f"📊 총 GPU 메모리: {total_memory_mb}MB, 제한: {self.max_memory_mb}MB")
            
        except Exception as e:
            self.logger.warning(f"❌ GPU 초기화 실패: {e}")
            self.gpu_available = False
    
    def get_gpu_memory_usage(self) -> dict:
        """현재 GPU 메모리 사용량 반환 (MB 단위)"""
        if not self.gpu_available:
            return {"used": 0, "free": 0, "total": 0, "utilization": 0}
        
        try:
            memory_info = pynvml.nvmlDeviceGetMemoryInfo(self.handle)
            utilization = pynvml.nvmlDeviceGetUtilizationRates(self.handle)
            
            return {
                "used": memory_info.used // (1024 * 1024),  # MB
                "free": memory_info.free // (1024 * 1024),  # MB  
                "total": memory_info.total // (1024 * 1024),  # MB
                "utilization": utilization.gpu  # GPU 사용률 %
            }
        except Exception as e:
            self.logger.error(f"GPU 메모리 사용량 조회 실패: {e}")
            return {"used": 0, "free": 0, "total": 0, "utilization": 0}
    
    def start_monitoring(self):
        """GPU 리소스 모니터링 시작"""
        if not self.gpu_available:
            self.logger.warning("⚠️ GPU가 사용 불가능하여 모니터링을 시작할 수 없습니다")
            return False
        
        if self.monitoring:
            self.logger.warning("⚠️ GPU 모니터링이 이미 실행 중입니다")
            return True
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info(f"🔍 GPU 모니터링 시작 (제한: {self.max_memory_mb}MB, 주기: {self.check_interval}초)")
        return True
    
    def stop_monitoring(self):
        """GPU 리소스 모니터링 중지"""
        if not self.monitoring:
            return
        
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        
        self.logger.info(f"🛑 GPU 모니터링 중지 (위반 횟수: {self.violation_count})")
    
    def _monitor_loop(self):
        """GPU 모니터링 메인 루프"""
        while self.monitoring:
            try:
                memory_info = self.get_gpu_memory_usage()
                used_memory = memory_info["used"]
                utilization = memory_info["utilization"]
                
                # 최대 사용량 업데이트
                if used_memory > self.max_memory_used:
                    self.max_memory_used = used_memory
                
                # 메모리 제한 초과 확인
                if used_memory > self.max_memory_mb:
                    self.violation_count += 1
                    self.logger.warning(f"🚨 GPU 메모리 제한 초과! 사용량: {used_memory}MB > 제한: {self.max_memory_mb}MB")
                    self.logger.warning(f"🚨 위반 횟수: {self.violation_count}, GPU 사용률: {utilization}%")
                    
                    # 콜백 호출
                    if self.on_memory_exceeded:
                        try:
                            self.on_memory_exceeded(used_memory, self.max_memory_mb, self.violation_count)
                        except Exception as e:
                            self.logger.error(f"메모리 초과 콜백 실행 오류: {e}")
                
                else:
                    # 정상 상태 로깅 (가끔씩만)
                    if self.violation_count % 12 == 0:  # 1분마다 (5초 * 12)
                        self.logger.debug(f"📊 GPU 상태: 메모리 {used_memory}MB/{memory_info['total']}MB, 사용률 {utilization}%")
                
                time.sleep(self.check_interval)
                
            except Exception as e:
                self.logger.error(f"GPU 모니터링 오류: {e}")
                if self.on_gpu_error:
                    try:
                        self.on_gpu_error(e)
                    except:
                        pass
                time.sleep(self.check_interval)
    
    def set_memory_exceeded_callback(self, callback: Callable):
        """메모리 초과 시 호출할 콜백 함수 설정"""
        self.on_memory_exceeded = callback
        self.logger.info("🔧 GPU 메모리 초과 콜백 설정됨")
    
    def set_gpu_error_callback(self, callback: Callable):
        """GPU 오류 시 호출할 콜백 함수 설정"""
        self.on_gpu_error = callback
        self.logger.info("🔧 GPU 오류 콜백 설정됨")
    
    def get_statistics(self) -> dict:
        """GPU 모니터링 통계 반환"""
        current_memory = self.get_gpu_memory_usage()
        return {
            "monitoring": self.monitoring,
            "gpu_available": self.gpu_available,
            "violation_count": self.violation_count,
            "max_memory_used": self.max_memory_used,
            "current_memory": current_memory,
            "memory_limit": self.max_memory_mb
        }
    
    def __del__(self):
        """소멸자 - 모니터링 정리"""
        self.stop_monitoring()
        if PYNVML_AVAILABLE and self.gpu_available:
            try:
                pynvml.nvmlShutdown()
            except:
                pass 