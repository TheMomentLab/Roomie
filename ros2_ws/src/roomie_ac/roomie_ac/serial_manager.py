import serial
import numpy as np
from . import config

class SerialManager:
    def __init__(self):
        self.port = config.SERIAL_PORT
        self.baud = config.SERIAL_BAUD_RATE
        self.timeout = config.SERIAL_TIMEOUT
        self.ser = None
        self.is_ready = False
        self.current_angles_deg = [0.0] * 4  # FK용 저장 각도

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print("🔌 시리얼 포트 연결됨. ESP32 부팅 대기 중")

            # 초기 홈 동기화
            print(f"🤝 초기 동기화: 홈 포지션 {config.HOME_POSITION_SERVO_DEG} 전송")
            final_angles = self.send_command(config.HOME_POSITION_SERVO_DEG)

            if final_angles is not None:
                print(f"✅ 연결 및 동기화 성공! 현재 각도: {final_angles}")
                self.is_ready = True
                return final_angles
            else:
                print("❌ 초기 동기화 실패")
                self.disconnect()
                return None

        except serial.SerialException as e:
            print(f"❌ 시리얼 연결 실패: {e}")
            return None

    def send_command(self, angles_deg):
        if not self.ser or not self.ser.is_open:
            print("🚫 시리얼 포트가 열려있지 않음")
            return None

        int_angles = np.round(angles_deg).astype(int)
        cmd = f"<M:{','.join(map(str, int_angles))}>"

        if config.DEBUG:
            print(f"  [SERIAL TX] -> {cmd}")

        try:
            self.ser.reset_input_buffer()
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
            self.current_angles_deg = int_angles.tolist()  # 현재 자세로 저장
            return self.current_angles_deg
        except serial.SerialException as e:
            print(f"💥 시리얼 송신 오류: {e}")
            self.disconnect()
            return None

    def get_current_angles_deg(self):
        return self.current_angles_deg

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 시리얼 포트 연결 해제됨")
        self.is_ready = False

