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
    ROOM_101: { ROOM_ID: roomParam, ENABLED_FEATURES: { food: true, supply: false, robot: true, history: true } },
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

const IS_DEV = true;
const BASE_URL = IS_DEV ? "http://localhost:8000" : "https://xxx.ngrok-free.app";
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

// 라우팅
function handleRouting() {
  const hash = location.hash.replace("#", "").split("&")[0];

  renderPageTemplate(hash); 

  switch (hash) {
    case "menu": food.loadFoodMenu(); break;
    case "cart": food.renderCart("cart-list", "btn-order"); break;
    case "status": food.renderOrderStatus("status-timeline"); break;
    case "supply": supply.loadSupplyMenu(); break;
    case "robot": robot.callRobot(); break;
    case "history": history.loadHistoryList(); break;
  }
}

function renderPageTemplate(route) {
  const container = document.querySelector(".main-content");
  if (!container) return;

  switch (route) {
    case "menu":
      container.innerHTML = `
        <h2>음식 주문</h2>
        <div id="food-menu-list"></div>
      `;
      break;

    case "cart":
      container.innerHTML = `
        <h2>장바구니</h2>
        <div id="cart-list"></div>
        <button id="btn-order">주문하기</button>
      `;
      break;

    case "status":
      container.innerHTML = `
        <h2>주문 상태</h2>
        <div id="status-timeline"></div>
      `;
      break;

    case "supply":
      container.innerHTML = `
        <h2>비품 요청</h2>
        <div id="supply-menu-list"></div>
      `;
      break;

    case "history":
      container.innerHTML = `
        <h2>요청 이력</h2>
        <div id="history-result"></div>
      `;
      break;

    default:
      container.innerHTML = `<p style="color:#888;">잘못된 페이지입니다.</p>`;
      break;
  }
}