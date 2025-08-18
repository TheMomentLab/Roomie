![Banner](https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/banner.png?raw=true)

<p align="center">
  <a href="https://www.apache.org/licenses/LICENSE-2.0">
    <img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=for-the-badge" alt="License">
  </a>
  <a href="https://docs.google.com/presentation/d/1ov9HC-qspBt8EVuyY66NAY0iMZ1nOyS5X9acsZ7iivs/edit?usp=sharing">
    <img src="https://img.shields.io/badge/PRESENTATION-GoogleSlides-yellow?style=for-the-badge&logo=google-slides&logoColor=white" alt="발표자료">
  </a>
  <a href="https://www.youtube.com/playlist?list=PLeVDEKHes6sHO5c1vp_Hu00HwNrdS69pk">
    <img src="https://img.shields.io/badge/DEMO-YouTube-red?style=for-the-badge&logo=youtube&logoColor=white" alt="데모영상">
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
- **프로젝트 목적**
  - 호텔에서 발생하는 반복적인 업무를 로봇이 자율적으로 수행하여</br>
    직원의 업무 부담을 줄이고</br>
    투숙객에게는 새롭고 편리한 경험을 제공</br>
  
- **프로젝트 기간**
  - 2025년 7월 7일 ~ 2025년 8월 13일(총 38일)
---

# 2. 주요 기능

## 🍽️ 룸서비스 배송 기능

<p align="center">
  <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery_demo.gif?raw=true" width="60%">
</p>

- **음식 주문 및 요청**
  - 객실 내 비치된 QR코드로 Guest GUI 접속
  - 메뉴 확인 후 주문 → Staff GUI로 알림 전송
  - 조리 완료 후 Staff GUI에서 “픽업 요청” 전달

- **픽업 및 적재**
  - Roomie가 레스토랑 픽업 위치로 이동
  - ArUco Marker 인식으로 정확한 픽업 위치 정렬
  - Robot GUI에 주문 내역 표시 → 적재 혼동 방지
  - 서랍 제어 (문 열림/잠금 센서, 적재 여부 감지 센서 포함)

- **객실 배송**
  - Nav2 기반 주행으로 객실 앞까지 이동
  - 목적지 QR/ArUco Marker 인식 → 객실 위치 확인
  - Guest GUI 및 Robot GUI를 통해 도착 알림 제공

- **음식 수령**
  - 고객이 Robot GUI 조작 → 서랍 해제 후 음식 수령
  - 완료 후 로봇은 대기 장소로 복귀

---

## 🧭 길 안내 기능

<p align="center">
  <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/guide_demo.gif?raw=true" width="60%">
</p>

- **안내 요청 및 목적지 입력**
  - 객실 카드 인증 시 목적지 자동 입력
  - Guest GUI 또는 Robot GUI에서 직접 입력 가능

- **안내 대상 인식**
  - 로봇 후면 카메라로 고객 인식
  - DeepSORT 기반 타겟 추적 알고리즘 적용

- **목적지 안내**
  - 고객을 일정 거리로 추적하며 목적지까지 안내
  - 고객이 시야에서 벗어나면 정지
  - 고객이 다시 나타나면 이동 재개

---

## 🏢 층간 이동 기능

<p align="center">
  <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator_demo.gif?raw=true" width="60%">
</p>

- **엘리베이터 호출**
  - Vision Service가 버튼 좌표 인식
  - Arm Controller를 이용해 호출 버튼 클릭

- **탑승 및 내부 조작**
  - 문 중앙 정렬 후 탑승
  - 층수 버튼 크기와 좌표를 기반으로 Arm Controller 제어
  - 상단 디스플레이 OCR로 목적 층 도착 여부 확인

- **하차**
  - 도착 후 중앙 정렬 → 안전하게 하차

---

## 📊 관리자 모니터링 기능

- **대시보드**
  - 현재 작업 수, 로봇 수를 실시간 확인
  - 2D 맵 상의 로봇 위치 표시

- **로봇 관리**
  - 현재 위치, 현재 작업, 배터리 상태 모니터링

- **작업 히스토리**
  - 작업 리스트 및 상세 기록 확인 가능
 
---

## ⚡️ micro-ROS ...
- **hello1**
  - hello2
  - hello3

---

# 3. 핵심 기술

## 1) 로봇암 제어

- **구성**
  - 서보모터, 2D 카메라, 버튼 클릭 엔드이펙터
- **좌표계 변환**
  - 베이스 좌표 → 팔끝 좌표 → 카메라 좌표 → 버튼 좌표 계산
- **동작 방식**
  - 관측 자세 → 클릭 준비 자세 → 버튼 클릭 → 클릭 확인
- **제어 기법**
  - Gaussian 속도/가속도 프로파일 적용 → 미세 진동(지터) 최소화

---

## 2) 경로 생성 및 주행

- **Nav2 기반 경로 계획**
  - 전역/지역 경로 생성 및 주행
- **웨이포인트 기반 경로 생성**
  - Depth 카메라로 장애물을 웨이포인트와 매칭
  - 최적 경로 계산에 **A\*** 알고리즘 적용
- **동적 장애물 처리**
  - 실시간 Depth 카메라 감지
  - 임계 거리 이내 접근 시 정지 → 사라지면 재개
- **RTR 주행 (Rotate–Translate–Rotate)**
  - 엘리베이터 탑승/하차 시 정밀 정렬 + 후진 동작 지원

---

## 3) 비전 인식

- **YOLOv8n 기반 객체 인식**
  - 장애물: 정적/동적/유리문
  - 엘리베이터: 버튼, 층수 표시기, 문, 방향등
- **성능 보완**
  - CNN으로 세부 버튼 분류
  - EasyOCR로 층수 인식
- **사람 타겟 추적**
  - YOLOv8n으로 사람 검출
  - DeepSORT로 특정 인물 추적 및 좌표 발행
 
---

## 4) 마이크로 ROS
- **hello1**
  - hello2
  - hello3

---

# 4. 기술적 문제 및 해결

### 🤖 로봇암 떨림 현상
- **문제**: 일반 등속 제어 시 팔끝이 미세하게 떨림  
- **해결**: Gaussian 함수 기반 속도/가속도 제어로 진동 최소화  

---

### 🛣️ 실내 복잡 환경 경로 생성
- **문제**: 통로가 좁거나 장애물 있을 때 우회 불가  
- **해결**: 웨이포인트 기반 경로 설계 + A\* 알고리즘으로 사전 우회 처리  

---

### 🧠 YOLO 단일 모델 한계
- **문제**: 객체 클래스 수 증가 → 성능 저하  
- **해결**: YOLOv8n은 ROI 생성, CNN으로 버튼 분류, EasyOCR로 층수 인식  

---

### 🏢 엘리베이터 탑승 정밀 동작
- **문제**: 버튼 클릭 및 내부 정렬 시 기존 Nav2 주행의 한계  
- **해결**: RTR(Rotate–Translate–Rotate) 패턴 적용 → 정밀 정렬 및 후진 동작 가능  

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


