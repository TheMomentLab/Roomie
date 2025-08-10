import socket
import cv2
import numpy as np

class UDPVideoStreamer:
    def __init__(self, host: str, port: int, logger=None, max_datagram_size: int = 60000):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.logger = logger
        self.max_datagram_size = max_datagram_size

    def send_frame_bgr(self, frame_bgr, resize_to=(640, 360), quality: int = 70):
        try:
            if frame_bgr is None:
                return False
            img = frame_bgr
            if resize_to is not None:
                img = cv2.resize(img, resize_to)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            if not ok:
                if self.logger:
                    self.logger.warn("JPEG 인코딩 실패")
                return False
            data = buf.tobytes()
            if len(data) > self.max_datagram_size:
                # 크면 품질 낮춰 재시도
                ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if not ok:
                    return False
                data = buf.tobytes()
                if len(data) > self.max_datagram_size and self.logger:
                    self.logger.warn(f"UDP 패킷이 큼({len(data)}B) → 전송 시 단편화 가능")
            self.sock.sendto(data, self.addr)
            return True
        except Exception as e:
            if self.logger:
                self.logger.warn(f"UDP 프레임 전송 실패: {e}")
            return False 