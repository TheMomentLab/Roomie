
import socket
import cv2
import numpy as np
import os
import time

class UDPVideoStreamer:
    def __init__(self, host: str = None, port: int = None, logger=None,
                 max_datagram_size: int = 60000,
                 target_ip: str = None, target_port: int = None,
                 max_fps: int = 15, quality: int = 70):

        ip = target_ip if target_ip is not None else host
        prt = target_port if target_port is not None else port

        ip = os.environ.get('VS_UDP_TARGET_IP', ip if ip is not None else '127.0.0.1')
        prt = int(os.environ.get('VS_UDP_TARGET_PORT', prt if prt is not None else 5005))
        self.addr = (ip, prt)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.logger = logger
        self.max_datagram_size = max_datagram_size
        self.max_fps = int(os.environ.get('VS_UDP_MAX_FPS', max_fps))
        self.quality = int(os.environ.get('VS_UDP_QUALITY', quality))
        self._last_ts = 0.0
        self._first_send_info_logged = False

    def send_frame_bgr(self, frame_bgr, resize_to=(640, 360), quality: int = None):
        try:
            if frame_bgr is None:
                return False

            # FPS 제한
            if self.max_fps > 0:
                now = time.time()
                min_dt = 1.0 / float(self.max_fps)
                if now - self._last_ts < min_dt:
                    return False # 스킵
                self._last_ts = now

            img = frame_bgr
            if resize_to is not None:
                img = cv2.resize(img, resize_to)
            q = self.quality if quality is None else quality
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if not ok:
                if self.logger:
                    self.logger.warn("JPEG 인코딩 실패")
                return False
            data = buf.tobytes()
            if len(data) > self.max_datagram_size:
                # 데이터가 너무 크면 품질을 낮춰서 재시도
                ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), max(40, q-10)])
                if not ok:
                    return False
                data = buf.tobytes()
                if len(data) > self.max_datagram_size and self.logger:
                    self.logger.warn(f"UDP 패킷이 너무 큽니다({len(data)}B). 전송 시 단편화될 수 있습니다.")
            self.sock.sendto(data, self.addr)
            if self.logger and not self._first_send_info_logged:
                self.logger.info(f"첫 UDP 프레임 전송: {len(data)} 바이트, 품질={q}, 대상={self.addr[0]}:{self.addr[1]}")
                self._first_send_info_logged = True
            return True
        except Exception as e:
            if self.logger:
                self.logger.warn(f"UDP 프레임 전송 실패: {e}")
            return False
