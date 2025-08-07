#!/usr/bin/env python3
"""
Roomie RC Configuration
로봇 컨트롤러 설정값들을 관리합니다.
"""

# ===== Debug Configuration =====
DEBUG_MODE = True  # True: 디버그 모드, False: 일반 모드
START_STATE = "delivery_complete"  # 시작 상태 설정
START_ROBOT_STATE = 13  # 시작 로봇 상태 설정

# ===== Robot Configuration =====
ROBOT_ID = 0
TASK_ID = 100
CURRENT_FLOOR_ID = 0  # 현재 층 (0: 1층)

# ===== Location Configuration =====
LOB_WAITING_LOCATION_ID = 0  # 로비 대기 위치 