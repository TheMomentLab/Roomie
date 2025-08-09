
import serial
import time
import numpy as np
from . import config

class SerialManager:
    def __init__(self):
        self.port = config.SERIAL_PORT
        self.baud = config.SERIAL_BAUD_RATE
        self.timeout = 0.1 # [수정] 비동기 방식이므로 타임아웃을 짧게 설정
        self.ser = None
        self.is_ready = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print("🔌 시리얼 포트 연결됨. ESP32 부팅 대기 중")
            time.sleep(2) # ESP32 부팅 시간 확보

            # [개선] 연결 확인을 위해 홈 포지션 명령을 보내지만, 응답은 기다리지 않음
            print(f"🤝 로봇과 통신 시작... 홈 포지션 {config.HOME_POSITION_SERVO_DEG}(으)로 이동 명령 전송.")
            success = self.send_command(config.HOME_POSITION_SERVO_DEG)
            
            if success:
                print("✅ 연결 및 초기화 명령 전송 성공!")
                self.is_ready = True
                return True # 이제 성공 여부(bool)만 반환
            else:
                print("❌ ESP32로 초기 명령 전송 실패.")
                self.disconnect()
                return False

        except serial.SerialException as e:
            print(f"❌ 시리얼 연결 실패: {e}")
            return False

    def send_command(self, angles_deg: np.ndarray) -> bool:
        """[최종 수정] ESP32에 각도 명령을 보내고, 응답을 기다리지 않습니다."""
        if not self.ser or not self.ser.is_open:
            print("🚫 시리얼 포트가 열려있지 않습니다.")
            return False

        int_angles = np.round(angles_deg).astype(int)
        cmd = f"<M:{','.join(map(str, int_angles))}>"
        
        if config.DEBUG: print(f"  [SERIAL TX] -> {cmd}")

        try:
            self.ser.write(cmd.encode('utf-8'))
            return True # 전송 성공 시 즉시 True 반환
        except serial.SerialException as e:
            print(f"💥 시리얼 쓰기 작업 중 오류 발생: {e}")
            self.disconnect()
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 시리얼 포트 연결 해제됨.")
        self.is_ready = False