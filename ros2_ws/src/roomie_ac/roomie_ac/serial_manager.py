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
            # 포트가 열리면 ESP32가 리셋되고 부팅될 시간을 줍니다.
            # 이 시간은 ESP32의 setup() 실행 시간보다 충분히 길어야 합니다.
            print("🔌 시리얼 포트 연결됨. ESP32 부팅 대기 중 (2.0초)...")
            time.sleep(2.0)
            
            # ======================= [핵심 수정] =======================
            # 1. 먼저 응답을 기다리는 대신, 초기 자세 명령을 보냅니다.
            print(f"🤝 로봇과 통신 시작... Python의 홈 포지션 {config.HOME_POSITION_SERVO_DEG}(으)로 동기화 명령을 보냅니다.")
            
            # 2. send_command는 명령 전송과 응답 수신을 모두 처리합니다.
            final_angles = self.send_command(config.HOME_POSITION_SERVO_DEG)
            # ==========================================================

            if final_angles is not None:
                print(f"✅ 연결 및 동기화 성공! 최종 확인된 각도: {final_angles}")
                self.is_ready = True
                return final_angles
            else:
                print("❌ ESP32로부터 초기 동기화 응답을 받지 못했습니다. 펌웨어와 통신 프로토콜을 확인하세요.")
                self.disconnect()
                return None

        except serial.SerialException as e:
            print(f"❌ 시리얼 연결 실패: {e}")
            return None

    def send_command(self, angles_deg):
        if not self.ser or not self.ser.is_open:
            print("🚫 시리얼 포트가 열려있지 않습니다.")
            return None

        # 정수형으로 변환하여 전송
        int_angles = np.round(angles_deg).astype(int)
        cmd = f"<M:{','.join(map(str, int_angles))}>"
        
        if config.DEBUG: print(f"  [SERIAL TX] -> {cmd}")

        try:
            self.ser.reset_input_buffer() # 명령 보내기 전, 수신 버퍼를 비워 이전 응답과의 혼선을 방지합니다.
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush() # 명령이 즉시 전송되도록 보장
            
            # 명령을 보낸 후, 상태 응답을 기다립니다.
            final_response = self.wait_for_status()

            # ======================= [디버깅 코드 추가] =======================
            if config.DEBUG:
                print(f"  ==> [DEBUG] send_command가 반환할 최종 값: {final_response}")
            # =================================================================

            return final_response
        except serial.SerialException as e:
            print(f"💥 시리얼 쓰기 작업 중 오류 발생: {e}")
            self.disconnect()
            return None


    def wait_for_status(self):
        if not self.ser: return None
        try:
            # 응답의 끝인 '>' 문자를 받을 때까지 기다립니다.
            response = self.ser.read_until(b'>').decode('utf-8')

            if config.DEBUG:
                # 수신된 raw 데이터를 그대로 출력하여 디버깅에 용이하게 합니다.
                print(f"  [SERIAL RX] <- '{response.strip()}'")

            if not response:
                print(f"❗️ ESP32로부터 응답 시간 초과({self.timeout}초). 연결 상태나 펌웨어를 확인하세요.")
                return None

            if response.startswith('<S:') and response.endswith('>'):
                # 정규 표현식을 사용하여 숫자만 정확히 추출합니다.
                nums_str = re.findall(r'-?\d+', response)
                if len(nums_str) == 4:
                    return np.array([int(n) for n in nums_str])
                else:
                    print(f"❓ 수신한 상태값의 개수가 4개가 아닙니다: {nums_str}")
                    return None

            elif '<ERR:' in response:
                error_code = re.findall(r'\d+', response)
                print(f"🚨 ESP32 오류 수신! 코드: {error_code[0] if error_code else 'N/A'}")
                return None
            
            else:
                # 예상치 못한 응답을 받았을 경우
                print(f"❓ ESP32로부터 알 수 없는 형식의 응답 수신: {response.strip()}")
                return None

        except Exception as e:
            print(f"💥 시리얼 응답 처리 중 예외 발생: {e}")
            return None


    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("🔌 시리얼 포트 연결 해제됨.")
        self.is_ready = False
