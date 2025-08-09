import serial
import time
import numpy as np
from . import config
import asyncio


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
            print("🔌 시리얼 포트 연결됨. ESP32 부팅 대기 중")
            time.sleep(2) 

            print(f"🤝 로봇과 통신 시작... 홈 포지션 {config.HOME_POSITION_SERVO_DEG}(으)로 이동 명령 전송.")
            success = self.send_command(config.HOME_POSITION_SERVO_DEG)
            
            if success:
                print("✅ 연결 및 초기화 명령 전송 성공!")
                self.is_ready = True
                return True
            else:
                print("❌ ESP32로 초기 명령 전송 실패.")
                self.disconnect()
                return False

        except serial.SerialException as e:
            print(f"❌ 시리얼 연결 실패: {e}")
            return False

    def send_command(self, angles_deg: np.ndarray) -> bool:
        if not self.ser or not self.ser.is_open:
            print("🚫 시리얼 포트가 열려있지 않습니다.")
            return False

        int_angles = np.round(angles_deg).astype(int)
        cmd = f"<M:{','.join(map(str, int_angles))}>"
        
        if config.DEBUG: print(f"  [SERIAL TX] -> {cmd}")
        
        try:
            self.ser.write(cmd.encode('utf-8'))
            return True
        except serial.SerialException as e:
            print(f"💥 시리얼 쓰기 작업 중 오류 발생: {e}")
            self.disconnect()
            return False

    async def wait_for_ack(self, timeout_sec: float) -> bool:
        """
        [수정됨] ESP32로부터 동작 완료 신호('<D>')를 비동기적으로 기다립니다.
        """
        if not self.ser or not self.ser.is_open:
            return False
        
        # [핵심 수정] 응답을 기다리기 직전에, 시리얼 입력 버퍼를 깨끗이 비웁니다.
        # 이렇게 하면 과거에 수신된 오래된 데이터(유령 응답)를 무시할 수 있습니다.
        self.ser.reset_input_buffer()
        
        start_time = time.time()
        buffer = b''
        while time.time() - start_time < timeout_sec:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                if b'<D>' in buffer:
                    if config.DEBUG: print("  [SERIAL RX] -> <D> (Done ACK 수신)")
                    return True
            await asyncio.sleep(0.01) 
        
        print(f"⌛️ ACK 대기 시간 초과 ({timeout_sec}초).")
        return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 시리얼 포트 연결 해제됨.")
        self.is_ready = False
