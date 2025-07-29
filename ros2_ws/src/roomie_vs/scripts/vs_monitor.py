#!/usr/bin/env python3
"""
Roomie VS 노드 시스템 모니터링 도구
실행 중인 vs_node의 리소스 사용량을 실시간으로 모니터링하고 
시스템 멈춤을 예방합니다.
"""

import psutil
import time
import subprocess
import sys
from datetime import datetime

class VSNodeMonitor:
    def __init__(self):
        self.vs_process = None
        self.max_memory_mb = 2500  # 2.5GB 메모리 제한
        self.max_cpu_percent = 80  # CPU 사용률 80% 제한
        self.check_interval = 3    # 3초마다 체크
        
    def find_vs_process(self):
        """vs_node 프로세스 찾기"""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if 'vs_node' in ' '.join(proc.info['cmdline']):
                    return psutil.Process(proc.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None
    
    def get_gpu_usage(self):
        """GPU 사용량 확인 (nvidia-smi 사용)"""
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=memory.used,memory.total', 
                                   '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if lines:
                    used, total = map(int, lines[0].split(', '))
                    return used, total, (used/total)*100
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
            pass
        return None, None, None
    
    def log_status(self, message, level="INFO"):
        """로그 출력"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        prefix = "⚠️" if level == "WARN" else "❌" if level == "ERROR" else "ℹ️"
        print(f"[{timestamp}] {prefix} {message}")
    
    def emergency_stop(self, reason):
        """비상 종료"""
        self.log_status(f"비상 종료 실행: {reason}", "ERROR")
        try:
            if self.vs_process and self.vs_process.is_running():
                # 먼저 SIGTERM으로 정상 종료 시도
                self.vs_process.terminate()
                time.sleep(3)
                
                # 여전히 실행 중이면 강제 종료
                if self.vs_process.is_running():
                    self.vs_process.kill()
                    self.log_status("강제 종료 완료", "WARN")
                else:
                    self.log_status("정상 종료 완료", "INFO")
            
            # OpenCV 윈도우 정리
            subprocess.run(['pkill', '-f', 'python.*vs_node'], 
                         stderr=subprocess.DEVNULL)
            
        except Exception as e:
            self.log_status(f"종료 중 오류: {e}", "ERROR")
    
    def monitor(self):
        """메인 모니터링 루프"""
        self.log_status("🔍 VS 노드 모니터링 시작")
        self.log_status(f"설정: 메모리 제한 {self.max_memory_mb}MB, CPU 제한 {self.max_cpu_percent}%")
        
        consecutive_high_usage = 0
        max_consecutive = 5  # 5회 연속 초과 시 종료
        
        try:
            while True:
                self.vs_process = self.find_vs_process()
                
                if not self.vs_process:
                    self.log_status("vs_node 프로세스를 찾을 수 없습니다. 10초 후 재시도...")
                    time.sleep(10)
                    continue
                
                try:
                    # 프로세스 정보 수집
                    memory_info = self.vs_process.memory_info()
                    memory_mb = memory_info.rss / 1024 / 1024  # MB 단위
                    cpu_percent = self.vs_process.cpu_percent(interval=1)
                    
                    # GPU 정보 수집
                    gpu_used, gpu_total, gpu_percent = self.get_gpu_usage()
                    
                    # 상태 출력
                    status_msg = f"PID:{self.vs_process.pid} | 메모리:{memory_mb:.1f}MB | CPU:{cpu_percent:.1f}%"
                    if gpu_used is not None:
                        status_msg += f" | GPU:{gpu_used}MB/{gpu_total}MB ({gpu_percent:.1f}%)"
                    
                    # 위험 수준 체크
                    is_dangerous = False
                    warnings = []
                    
                    if memory_mb > self.max_memory_mb:
                        warnings.append(f"메모리 과사용 ({memory_mb:.1f}MB > {self.max_memory_mb}MB)")
                        is_dangerous = True
                    
                    if cpu_percent > self.max_cpu_percent:
                        warnings.append(f"CPU 과사용 ({cpu_percent:.1f}% > {self.max_cpu_percent}%)")
                        is_dangerous = True
                    
                    if gpu_percent and gpu_percent > 90:
                        warnings.append(f"GPU 메모리 과사용 ({gpu_percent:.1f}%)")
                        is_dangerous = True
                    
                    if is_dangerous:
                        consecutive_high_usage += 1
                        self.log_status(f"{status_msg} | 경고: {', '.join(warnings)} | 연속: {consecutive_high_usage}/{max_consecutive}", "WARN")
                        
                        if consecutive_high_usage >= max_consecutive:
                            self.emergency_stop(f"연속 {consecutive_high_usage}회 리소스 초과")
                            break
                    else:
                        consecutive_high_usage = 0
                        self.log_status(status_msg)
                    
                    time.sleep(self.check_interval)
                    
                except psutil.NoSuchProcess:
                    self.log_status("vs_node 프로세스가 종료되었습니다")
                    break
                    
                except Exception as e:
                    self.log_status(f"모니터링 중 오류: {e}", "ERROR")
                    time.sleep(self.check_interval)
                    
        except KeyboardInterrupt:
            self.log_status("사용자에 의해 모니터링이 중단되었습니다")
        except Exception as e:
            self.log_status(f"예상치 못한 오류: {e}", "ERROR")
        finally:
            self.log_status("🏁 모니터링 종료")

def main():
    print("🛡️ Roomie VS 노드 모니터링 시작")
    print("Ctrl+C로 종료하세요")
    print("-" * 50)
    
    monitor = VSNodeMonitor()
    monitor.monitor()

if __name__ == "__main__":
    main() 