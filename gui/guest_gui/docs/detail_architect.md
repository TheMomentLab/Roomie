📁 폴더 구조 설계 (guest_gui 기준)
bash
복사
편집
guest_gui/
├── index.html                   # 메인 엔트리 포인트 (모든 기능 포함 SPA)
├── js/
│   ├── app.js                   # 메인 앱 로직 (렌더링/이벤트 처리)
│   ├── common.js                # 공용 유틸 및 API 통신 함수
│   ├── food.js                  # 음식 관련 로직
│   ├── supply.js                # 비품 관련 로직
│   ├── robot.js                 # 호출 관련 로직
│   └── history.js               # 요청 이력 조회 로직
├── css/
│   └── common.css               # 통합 스타일
├── assets/
│   ├── images/
│   │   └── background_1.png     # 공통 배경 이미지
│   └── fonts/                   # (선택) 사용자 정의 폰트 등
├── docs/
│   ├── message.md               # 전체 메시지 명세서
│   └── detail_architect.md      # 전체 메시지 명세서

└── README.md                    # 개발자용 설명 문서
⚙️ index.html 구조 요약
SPA(Single Page Application) 스타일로 구성하여, QR 진입 시 config를 기반으로 기능 필터링하여 동작합니다.

<!-- 필수 영역 -->
<head>
  <link rel="stylesheet" href="css/common.css" />
  <script src="config/room_201.js"></script> <!-- QR로 로딩 -->
  <script src="js/app.js" type="module"></script>
</head>

<body>
  <header>
    <h1 id="room-name">ROOM_201</h1>
  </header>

  <main id="main-ui">
    <!-- 음식 영역 -->
    <section id="food-section">
      <h2>음식 주문</h2>
      <div id="food-menu-list"></div>
    </section>

    <!-- 비품 영역 -->
    <section id="supply-section">
      <h2>비품 요청</h2>
      <div id="supply-menu-list"></div>
    </section>

    <!-- 로봇 호출 -->
    <section id="robot-section">
      <h2>로봇 호출</h2>
      <button id="btn-call-robot">호출하기</button>
    </section>

    <!-- 요청 이력 -->
    <section id="history-section">
      <h2>요청 이력 조회</h2>
      <div id="history-result"></div>
    </section>
  </main>

  <footer>
    <p>Room Service Assistant</p>
  </footer>

  <div id="global-toast" class="toast hidden"></div>
</body>
⚙️ config/room_201.js 예시
js
복사
편집
const ROOM_ID = "ROOM_201";
const API_URL = "http://<SERVER_IP>:<PORT>";
const ENABLED_FEATURES = {
  food: true,
  supply: true,
  robot: true,
  history: true,
};
🔁 페이지 동작 방식 (app.js)

document.addEventListener("DOMContentLoaded", () => {
  initPage();     // header 및 기능 필터링 렌더링
  bindEvents();   // 각 버튼의 클릭 이벤트 등록
});
📌 initPage()
ROOM_ID를 기반으로 헤더와 기능 활성화

ENABLED_FEATURES에서 false인 섹션은 숨김 처리

ex: 로비는 food/supply 숨김

📌 bindEvents()
버튼	동작 설명
음식 주문 버튼	/api/gui/get_food_menu 호출 후 렌더링
비품 요청 버튼	/api/gui/get_supply_menu 호출 후 렌더링
로봇 호출 버튼	/api/gui/create_call_task POST 요청
요청 이력 버튼	/api/gui/get_call_history 또는 /get_order_history 호출

📌 응답 처리 예시
js

function renderFoodMenu(items) {
  const container = document.getElementById("food-menu-list");
  container.innerHTML = items.map(item => `
    <div class="card">
      <img src="${item.image}" />
      <p>${item.food_name}</p>
      <p>${item.price.toLocaleString()}원</p>
    </div>
  `).join('');
}
🔔 WebSocket 알림 처리 흐름 (선택)
js
복사
편집
const socket = new WebSocket(`ws://${HOST}/api/gui/ws/admin/${ROOM_ID}`);
socket.onmessage = (event) => {
  const data = JSON.parse(event.data);
  handleWebSocketEvent(data);
};

function handleWebSocketEvent(event) {
  switch (event.action) {
    case "call_request_acceptance":
      showToast(`호출이 수락되었습니다! 예상 대기: ${event.payload.estimated_wait_time}분`);
      break;
    case "robot_arrival_completion":
      showToast("로봇이 도착했습니다!");
      break;
    // ...
  }
}
📦 각 API 연동 함수 (common.js)
js
복사
편집
export async function sendApiRequest(endpoint, payload) {
  const response = await fetch(`${API_URL}${endpoint}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  return result.payload;
}
🎨 CSS 기본 구조 (common.css)
css
복사
편집
body {
  background: url('../assets/images/background_1.png') no-repeat center center;
  background-size: cover;
}

.toast {
  position: fixed;
  bottom: 10px;
  left: 50%;
  transform: translateX(-50%);
  background: #000;
  color: #fff;
  padding: 10px;
  border-radius: 8px;
}

.toast.hidden {
  display: none;
}
🧭 기능 정리표
기능 항목	API Endpoint	사용 위치
음식 메뉴 조회	/api/gui/get_food_menu	객실
비품 메뉴 조회	/api/gui/get_supply_menu	객실
로봇 호출	/api/gui/create_call_task	객실, 로비 등
요청 이력 조회	/api/gui/get_order_history, /get_call_history	전 구역
알림 (WebSocket)	/api/gui/ws/admin/{location_name}	전 구역 (선택)

✅ 마무리 정리
기존 page_callfood_1~4.html은 → index.html + config.js로 통합 가능

기능별 로직 분리는 JS 모듈화

배경, 메뉴 구조는 완전 재사용

QR 기반이므로 config/room_xxx.js만 바꾸면 모든 UI 자동 변경