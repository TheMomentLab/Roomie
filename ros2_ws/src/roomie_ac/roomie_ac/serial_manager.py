import serial
import time
import re
import numpy as np  
from . import config


class SerialManager:
    def __init__(self):
        self.port = config.SERIAL_PORT
        self.baud = config.SERIAL_BAUD_RATE
        self.timeout = config.SERIAL_TIMEOUT
        self.ser = None
        self.is_ready = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print(f"✅ 시리얼 포트 연결됨: {self.port}. ESP32 초기화 대기 중...")
            time.sleep(3)
            self.ser.reset_input_buffer() 
            
            print("🤝 로봇과 통신 시작(Handshake)...")
            initial_angles = self.send_command(config.HOME_POSITION_SERVO_DEG) 

            if initial_angles is not None:
                print(f"🤝 handshake 성공! 초기 서보 각도 수신: {initial_angles}")
                self.is_ready = True
                return initial_angles
            else:
                print("❌ ESP32로부터 초기 상태 응답을 받지 못했습니다.")
                self.disconnect()
                return None
        except serial.SerialException as e:
            print(f"❌ 시리얼 연결 실패: {e}")
            return None

    def send_command(self, angles_deg):
        if not self.ser or not self.ser.is_open:
            print("🚫 시리얼 포트가 열려있지 않습니다.")
            return None

        cmd = f"<M:{','.join(map(str, angles_deg))}>"
        if config.DEBUG: print(f"  [SERIAL TX] -> {cmd}")

        self.ser.write(cmd.encode())
        self.ser.flush()
        return self.wait_for_status()

    def wait_for_status(self):
        if not self.ser: return None
        try:
            response = self.ser.read_until(b'>').decode('utf-8')

            if config.DEBUG: 
                print(f"  [SERIAL RX] <- {response.strip()}")

            if not response:
                print("❗️ ESP32로부터 응답을 받지 못했습니다.")
                return None
            
            if response.startswith('<S:') and response.endswith('>'): 
                nums = re.findall(r'\d+', response)
                return np.array([int(n) for n in nums])
            
            elif '<ERR:' in response: 
                error_code = re.findall(r'\d+', response)
                print(f"🚨 ESP32 오류 수신! 코드: {error_code[0] if error_code else 'N/A'}")
                return None
            
        except Exception:
            return None


    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 시리얼 포트 연결 해제됨.")
        self.is_ready = False