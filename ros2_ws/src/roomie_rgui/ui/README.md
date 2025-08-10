# Roomie RGUI UI 가이드 (images/qrc)

- 위치: `ros2_ws/src/roomie_rgui/ui`
- 목적: UI(.ui) 화면에서 이미지 리소스를 안정적으로 표시하기 위한 규칙과 절차 정리

## 디렉토리 구조
- `ui/guide`, `ui/delivery`, `ui/elevator`, `ui/common`, `ui/countdown` 등: Qt Designer `.ui` 파일
- `roomie_rgui/assets`: 이미지 리소스 저장소 (qrc에 등록하여 사용)
- `roomie_rgui/assets/resources.qrc`: Qt 리소스 목록(qrc)
- `roomie_rgui/ui_controllers/*.py`: 화면별 컨트롤러(일부 화면은 동적 이미지 로딩 보조)

## 이미지 로딩 정책
- 1순위: `.ui`에서 qrc 경로로 직접 지정
  - 예) `:/roomie_rgui/assets/rgui_guide_1.png`
  - `.ui` 파일 하단에 `resources.qrc` include 필요
- 2순위(보조): 컨트롤러에서 동적 로딩
  - 가이드 관련 일부 화면은 컨트롤러가 qrc → 절대경로 폴백으로 이미지를 보장 로드
  - 절대경로 폴백: `/home/jinhyuk2me/project_ws/Roomie/ros2_ws/src/roomie_rgui/roomie_rgui/assets/…`

현재 사용되는 가이드 관련 이미지(assets):
- `rgui_guide_1.png` (GUIDE_REQUEST)
- `rgui_card.png`, `rgui_touch.png` (INPUT_METHOD_SELECTION)
- `rgui_scan.png` (CARD_KEY_WAITING)
- `rgui_guide_out.png` (RECHECKING; 컨트롤러가 qrc 로딩)
- `rgui_eye_2.png` (DESTINATION_ARRIVED; .ui에서 qrc 로딩)

## 새 이미지를 추가하는 방법
1) 파일 추가
   - 이미지를 `roomie_rgui/assets` 아래에 복사
2) qrc 등록
   - `roomie_rgui/assets/resources.qrc`에 `<file>파일명.png</file>` 추가
3) .ui에 연결 (권장)
   - `.ui` 하단에 `resources.qrc` include 추가
   - 대상 `QLabel`의 `pixmap` 속성을 `:/roomie_rgui/assets/파일명.png`로 설정
4) (선택) 컨트롤러 보조 로딩 사용
   - 가이드 화면의 경우 컨트롤러가 동일 라벨명에 대해 qrc → 절대경로 폴백으로 로딩함
   - 라벨명 예: `roadImage`, `cardKeyImage`, `directInputImage`

## 빌드/실행
- qrc/이미지 추가 후에는 반드시 빌드 필요
  - 워크스페이스 루트에서:
    - 일반: `colcon build`
    - 문제가 지속되면: `rm -rf build install log && colcon build`
- 실행: `ros2 run roomie_rgui rgui_node`

## 트러블슈팅
- 화면에 이미지가 안 보임
  - 이미지가 `roomie_rgui/assets`에 존재하는지 확인
  - `resources.qrc`에 파일이 등록되었는지 확인
  - `.ui`에 `resources.qrc` include가 있는지 확인
  - `.ui` 라벨 `pixmap`이 qrc 경로로 설정되었는지 확인
  - 빌드/재실행(`colcon build` 후 `ros2 run …`)
- 가이드 화면 전환이 안됨
  - GUIDE_REQUEST는 카드 영역(`touchButton`) 터치 또는 전체 화면 터치로 `INPUT_METHOD_SELECTION`로 이동되도록 컨트롤러에서 보장
- 선택 버튼 누락 경고
  - `CARD_KEY_WAITING`의 `cancelButton`은 선택 사항이며 없는 경우 경고 없이 동작

## 참고
- 배송 화면들은 대부분 `.ui`에서 qrc 픽스맵을 직접 지정 (예: `DELI_1_PICKUP_MOVING.ui`)
- 가이드 화면들은 `.ui` qrc + 컨트롤러 보조 로딩을 혼합 사용하여 설치 경로/리소스 누락에도 표시되도록 설계 