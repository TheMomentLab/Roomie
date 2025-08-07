// app.js
import * as food from "./food.js";
import * as supply from "./supply.js";
import * as robot from "./robot.js";
import * as history from "./history.js";

// 1. 룸 정보 매핑
function getConfigFromRoomParam() {
  const roomParam = new URLSearchParams(window.location.search).get("room") || "ROOM_201";

  const configMap = {
    ROOM_201: { ROOM_ID: roomParam, ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
    ROOM_102: { ROOM_ID: roomParam, ENABLED_FEATURES: { food: true, supply: false, robot: true, history: true } },
    LOB_CALL: { ROOM_ID: roomParam, ENABLED_FEATURES: { food: false, supply: false, robot: true, history: true } },
    RES_CALL: { ROOM_ID: roomParam, ENABLED_FEATURES: { food: true, supply: false, robot: true, history: false } }
  };

  return configMap[roomParam] || {
    ROOM_ID: roomParam,
    ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true },
  };
}

const CONFIG = getConfigFromRoomParam();
const ROOM_ID = CONFIG.ROOM_ID;
const ENABLED_FEATURES = CONFIG.ENABLED_FEATURES;
window.ROOM_ID = ROOM_ID;
window.CONFIG = CONFIG;  
const IS_DEV = true;
const BASE_URL = IS_DEV ? "http://localhost:8072" : "https://xxx.ngrok-free.app";
window.API_URL = `${BASE_URL}/api/gui`;
window.WS_URL = `${BASE_URL.replace("http", "ws")}/api/gui/ws/guest/${ROOM_ID}`;

const LOCATION_NAME_MAP = {
  "ROOM_101": "101호",
  "ROOM_102": "102호",
  "ROOM_201": "201호",
  "ROOM_202": "202호",
  "LOB_WAITING": "로비 대기",
  "LOB_CALL": "로비 호출",
  "RES_PICKUP": "레스토랑 픽업",
  "RES_CALL": "레스토랑 호출",
  "SUP_PICKUP": "비품 픽업",
  "ELE_1": "엘리베이터 1층",
  "ELE_2": "엘리베이터 2층"
};

function getDisplayName(locationName) {
  return LOCATION_NAME_MAP[locationName] || locationName;
}

document.addEventListener("DOMContentLoaded", () => {
  initPage();
  bindEvents();
  handleRouting();
});

window.addEventListener("hashchange", handleRouting);

const routes = {
  "/": () => renderHomeView(),
  "/food": () => renderFoodMenuView(),
  "/supply": () => renderSupplyMenuView(),
  "/call_waiting": () => renderRobotWaitingView(),
  "/call_ready": () => renderCallReadyView(),
  "/task_list": () => renderTaskListView(),
  "/history_detail": () => renderHistoryDetailView(),
};


// 초기 페이지 설정
function initPage() {
  const roomNameEl = document.getElementById("room-name");
  if (roomNameEl) roomNameEl.textContent = getDisplayName(ROOM_ID);

  const sectionMap = {
    food: document.getElementById("btn-order-food"),
    supply: document.getElementById("btn-order-supply"),
    robot: document.getElementById("btn-call-robot"),
    history: document.getElementById("btn-order-history"),
  };

  for (const [key, el] of Object.entries(sectionMap)) {
    if (el) el.style.display = ENABLED_FEATURES[key] ? "block" : "none";
  }
}

// 버튼 이벤트 바인딩
function bindEvents() {
  const foodBtn = document.getElementById("btn-order-food");
  const supplyBtn = document.getElementById("btn-order-supply");
  const robotBtn = document.getElementById("btn-call-robot");
  const historyBtn = document.getElementById("btn-order-history");

  if (foodBtn) foodBtn.addEventListener("click", () => location.hash = "menu");
  if (supplyBtn) supplyBtn.addEventListener("click", () => location.hash = "supply");
  if (robotBtn) robotBtn.addEventListener("click", () => location.hash = "robot");
  if (historyBtn) historyBtn.addEventListener("click", () => location.hash = "history");
}

function handleRouting() {
  const hash = location.hash.replace("#", "").split("&")[0];
  
  if (!hash) {
    return;
  }

  // 1. main-content를 비우고 새로운 템플릿을 렌더링합니다.
  renderPageTemplate(hash); 

  // 2. 렌더링된 템플릿에 내용을 채우는 스크립트를 실행합니다.
  switch (hash) {
    case "menu":
      food.loadFoodMenu("food-menu-list"); // 음식 메뉴 로드
      break;
    case "cart":
      food.renderCart("cart-list", "btn-order"); // 장바구니 렌더링
      break;
    case "order-success":
      food.renderOrderStatus("estimated-time"); // 주문 성공 화면 렌더링
      break;
    case "supply":
      supply.loadSupplyMenu("supply-menu-list"); // 비품 메뉴 로드
      break;
    case "robot-success": // 로봇 호출 성공 시 음식 주문 성공 화면 재사용
      robot.renderRobotStatus("estimated-time");
      break;
    case "history":
      history.loadHistoryList("history-result"); // 요청 이력 목록 로드
      break;
    case "history-detail":
      history.renderHistoryDetail("history-detail-container"); // 요청 이력 상세 로드
      break;
    case "call-robot":
      robot.renderRobotCalling(); // 로봇 호출 중 뷰
      break;
    default:
      // 해시가 없거나 일치하는 것이 없으면 메인 화면으로 돌아갑니다.
      // (index.html의 기본 내용이 표시되도록 main-content를 비웁니다)
      const container = document.querySelector(".main-content");
      if (container) container.innerHTML = '';
      break;
  }
}
// [수정] 각 페이지의 실제 HTML 구조를 템플릿으로 추가
function renderPageTemplate(route) {
  const container = document.querySelector(".main-content");
  if (!container) return;

  // 페이지가 변경될 때마다 이전 클래스를 지우고 새 클래스를 추가합니다.
  container.className = 'main-content'; // 기본 클래스로 초기화
  container.classList.add(`view-${route || 'home'}`); // 예: view-menu, view-cart

  let template = '';

  switch (route) {
    case "menu": // 음식 주문 메뉴 (page_callfood_1.html 기반)
      template = `
        <div class="back-button" onclick="history.back()"></div>
        <div class="menu-list" id="food-menu-list">
          </div>
        <footer class="footer">
            <span class="total-price" id="total_price">총 0원</span>
            <div class="divider"></div>
            <button id="button_cart" class="cart-button" onclick="location.hash='cart'">장바구니 보기</button>
        </footer>
      `;
      break;

    case "cart": // 장바구니 (page_callfood_2.html 기반)
      template = `
        <div class="cart-header">
            <img class="back-button" src="asset/v14_863.png" alt="뒤로가기" onclick="history.back()">
            <h2 class="cart-title">주문상품</h2>
        </div>
        <div id="cart-list" class="cart-items">
          </div>
        <div class="footer">
            <div class="notice">
                장바구니에 담긴 상품은 최대 10분 보관됩니다.
            </div>
            <div class="total-price-container">
                <span>상품금액</span>
                <span id="total-price-display">0원</span>
            </div>
            <button id="btn-order" class="order-button">주문하기</button>
        </div>
      `;
      break;

    case "order-success": // 주문/호출 성공 (page_callfood_3.html 기반)
    case "robot-success":
      template = `
        <button class="order-history-button" onclick="location.hash='history'">주문 내역</button>
        <div class="status-section">
            <h1 class="status-text">요청이<br>접수되었습니다.</h1>
            <div class="robot-icon-container">
                <img src="asset/v15_67.png" alt="배달 로봇">
                <div class="line"></div>
            </div>
        </div>
        <div class="time-section">
            <span>예상 대기 시간</span>
            <span id="estimated-time" class="time-value"></span>
        </div>
        <button class="close-button" onclick="location.hash=''">닫기</button>
      `;
      break;

    case "supply": // 비품 주문 (음식 주문과 동일한 템플릿 사용)
      template = `
        <div class="back-button" onclick="history.back()"></div>
        <div class="menu-list" id="supply-menu-list">
            </div>
        <footer class="footer">
            <span class="total-price" id="total_price">총 0원</span>
            <div class="divider"></div>
            <button id="button_cart" class="cart-button" onclick="alert('비품은 장바구니 없이 바로 요청됩니다.')">비품 요청하기</button>
        </footer>
      `;
      break;

    case "history": // 요청 이력 목록
      template = `
        <div class="cart-header">
            <img class="back-button" src="asset/v14_863.png" alt="뒤로가기" onclick="location.hash=''">
            <h2 class="cart-title">요청 이력</h2>
        </div>
        <div id="history-result" class="history-items"></div>
      `;
      break;

    case "history-detail": // 요청 이력 상세 (page_callfood_4.html 기반)
      template = `
        <div class="status-card" id="history-detail-container">
            </div>
      `;
      break;

    default:
      // 해시가 없는 경우, main-content를 비워 초기 화면을 보여줍니다.
      template = '';
      break;
  }
  container.innerHTML = template;
}