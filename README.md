![Banner](https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/banner.png?raw=true)

<p align="center">
  <a href="https://www.apache.org/licenses/LICENSE-2.0">
    <img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=for-the-badge" alt="License">
  </a>
  <a href="https://docs.google.com/presentation/d/1ov9HC-qspBt8EVuyY66NAY0iMZ1nOyS5X9acsZ7iivs/edit?usp=sharing">
    <img src="https://img.shields.io/badge/PRESENTATION-GoogleSlides-yellow?style=for-the-badge&logo=google-slides&logoColor=white" alt="발표자료">
  </a>
  <a href="https://youtube.com/playlist?list=PLCGG9KRfKwMmQqXvp43pChNMyyLSyjHp9&si=Qhge_jErHH7Qlb4e">
    <img src="https://img.shields.io/badge/DEMO-YouTube-red?style=for-the-badge&logo=youtube&logoColor=white" alt="배송기능 데모영상">
  </a>
</p>

# 📚 목차

- [1. 프로젝트 개요](#1-프로젝트-개요)
- [2. 주요 기능](#2-주요-기능)
- [3. 핵심 기술](#3-핵심-기술)  
- [4. 기술적 문제 및 해결](#4-기술적-문제-및-해결)
- [5. 시스템 설계](#5-시스템-설계)
- [6. 프로젝트 구조](#6-프로젝트-구조)
- [7. 기술 스택](#7-기술-스택)
- [8. 일정 관리](#8-프로젝트-일정-관리)
- [9. 팀 구성](#9-팀-구성)
- [10. 라이선스](#10-라이선스)

---

# 1. 프로젝트 개요

---

# 2. 주요 기능

---

# 3. 핵심 기술

---

# 4. 기술적 문제 및 해결

---

# 5. 시스템 설계

---

# 6. 프로젝트 구조

```
Roomie/
├── ros2_ws/                            # ROS2 공통 워크스페이스
│   ├── build/                          # colcon build 시 자동 생성
│   ├── install/
│   ├── log/
│   └── src/
│       ├── micro_ros_setup/           # micro-ros 빌드 도구
│       ├── roomie_msgs/               # 공용 메시지 (msg/srv/action 정의)
│       ├── roomie_rc/                 # 로봇 제어 노드 (RC)
│       ├── roomie_rgui/               # 로봇 GUI 노드 (RGUI)
│       ├── roomie_vs/                 # Vision Service 노드 (VS)
│       ├── roomie_rms/                # Main Server 노드 (RMS)
│       ├── roomie_agui/               # 관리자 GUI 노드 (Admin GUI)
│       ├── roomie_ac/                 # Arm Controller 노드 (AC)
│       └── bringup/                   # 통합 launch 파일 모음
│
├── esp32_firmware/                     # Micro-ROS 전용 ESP32 펌웨어 개발
│   ├── arm_unit/                      # Arm 서보 제어용 펌웨어
│   │   └── src/
│   └── io_controller/                 # 센서, 서랍, LED 제어
│       └── src/
│
├── gui/                               # GUI 애플리케이션들 (비 ROS)
│   ├── staff_gui/                     # 직원용 GUI
│   └── guest_gui/                     # 투숙객용 GUI
│
├── assets/                            # 이미지 및 리소스 파일
│   └── images/
│
├── docs/                              # 설계 문서
│   ├── architecture/                  # 시스템 아키텍처
│   ├── interface.md                   # 통신 인터페이스 정의
│   └── state_diagram/                 # 상태 다이어그램
│
├── .gitignore
├── README.md
└── LICENSE
```

---

# 7. 기술 스택

---

# 8. 프로젝트 관리

---

# 9. 팀 구성

---

# 10. 라이선스

---


