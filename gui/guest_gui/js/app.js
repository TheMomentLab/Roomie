// app.js

import * as order from "./order.js";
import * as robot from "./robot.js";
import * as history from "./history.js";
import { initWebSocket } from "./common.js";

// --- 전역 설정 ---
function getConfigFromRoomParam() {
  const roomParam = new URLSearchParams(window.location.search).get("room") || "ROOM_101";
  const configMap = {
    ROOM_201: { ROOM_ID: "ROOM_201", ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
    ROOM_101: { ROOM_ID: "ROOM_101", ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
    ROOM_102: { ROOM_ID: "ROOM_102", ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
    LOB_CALL: { ROOM_ID: "LOB_CALL", ENABLED_FEATURES: { food: false, supply: false, robot: true, history: true } },
    RES_CALL: { ROOM_ID: "RES_CALL", ENABLED_FEATURES: { food: false, supply: false, robot: true, history: true } }
  };
  return configMap[roomParam] || configMap["ROOM_101"];
}


// index.html폴더에서 실행
// python3 -m http.server 8000        
//http://127.0.0.1:8000/
// 자기 ip로 수정
// http://192.168.0.8:8000/?room=ROOM_102 
const CONFIG = getConfigFromRoomParam();
const ROOM_ID = CONFIG.ROOM_ID;
const ENABLED_FEATURES = CONFIG.ENABLED_FEATURES;
window.ROOM_ID = ROOM_ID;
const IS_DEV = true;
const BASE_URL = IS_DEV ? " https://133ee228c163.ngrok-free.app" : "https://roomie.com";
//const BASE_URL = IS_DEV ? "http://0.0.0.0:8888" : "https://roomie.com";
window.API_URL = `${BASE_URL}/api/gui`;
window.WS_URL = `${BASE_URL.replace("http", "ws")}/api/gui/ws/guest/${ROOM_ID}`;

// --- 앱 상태 관리 ---
let currentOrderType = 'food';

// --- 앱 초기화 ---
document.addEventListener("DOMContentLoaded", () => {
  initPage();
  bindEvents();
  handleRouting();
  initWebSocket();
});

window.addEventListener("hashchange", handleRouting);

function initPage() {
  const roomNameEl = document.getElementById("room-name");
  if (roomNameEl) roomNameEl.textContent = ROOM_ID;

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

// --- 이벤트 바인딩 ---
function bindEvents() {
    document.body.addEventListener('click', (e) => {
        const target = e.target.closest('button');
        if (!target) return;

        const targetId = target.id;
        if (targetId === 'btn-order-food') {
            currentOrderType = 'food';
            location.hash = 'menu';
        } else if (targetId === 'btn-order-supply') {
            currentOrderType = 'supply';
            location.hash = 'menu';
        } else if (targetId === 'btn-call-robot') {
            robot.callRobot();
        } else if (targetId === 'btn-order-history') {
            location.hash = 'history';
        }
    });
}

// --- 라우팅 및 렌더링 ---
function handleRouting() {
    const hash = location.hash.replace("#", "") || "/";
    renderPageTemplate(hash);
}
function renderPageTemplate(route) {
    const container = document.querySelector(".main-content");
    if (!container) return;
    
    const mainRoute = route.split('&')[0];
    const queryString = route.includes('&') ? route.substring(route.indexOf('&') + 1) : '';
    const params = new URLSearchParams(queryString);
    
    const mainHeader = document.querySelector('.header');
    let template = '';
    
    const backButton = `<button class="header-btn back-button" onclick="history.back()"></button>`;
    const placeholder = `<div class="header-placeholder"></div>`;

    // --- 라우트별 템플릿 분기 처리 ---
    if (mainRoute === '/') {
        container.className = 'main-content main-content-home';
        if(mainHeader) mainHeader.style.display = 'block';

        // ✅ [수정] ENABLED_FEATURES 설정에 따라 버튼을 조건부로 렌더링합니다.
        template = `
            <h1>ROOMIE</h1>
            <p class="subtitle">호텔 안내 & 주문 배송 서비스</p>
            <p class="welcome-message">ROOMIE에 오신 걸 환영합니다.</p>
            <div class="button-grid">
                ${ENABLED_FEATURES.robot ? `<button id="btn-call-robot" class="grid-button">로봇호출</button>` : ''}
                ${ENABLED_FEATURES.history ? `<button id="btn-order-history" class="grid-button">요청조회</button>` : ''}
                ${ENABLED_FEATURES.food ? `<button id="btn-order-food" class="grid-button">음식주문</button>` : ''}
                ${ENABLED_FEATURES.supply ? `<button id="btn-order-supply" class="grid-button">비품주문</button>` : ''}
            </div>
        `;
    } else {
        // --- 흰색 박스가 있는 서브 페이지 템플릿 ---
        container.className = 'main-content main-content-sub';
        if(mainHeader) mainHeader.style.display = 'none';
        
        let pageContent = '';
        switch (mainRoute) {
            // =================================================================
            // ✅ 1. 로봇 호출 로딩 페이지 수정
            // =================================================================
            case 'call-robot-loading':
                pageContent = `
                    <div class="box-header">
                        ${placeholder}
                        <h2 class="page-title">로봇 호출</h2>
                        ${placeholder}
                    </div>
                    <div class="status-section vertical-center">
                        <p class="status-text" style="font-size: 24px; margin-bottom: 20px;">호출 가능한 로봇이<br>있는지 확인중입니다.</p>
                        <div class="loading-spinner"></div>
                    </div>
                    <button class="btn-bottom-close" onclick="location.hash='/'">취소</button>
                `;
                break;

            case "robot-accepted": { // ⬅️ 여는 중괄호 추가
                const taskName = params.get('task');
                const waitTime = params.get('wait');

                const historyDetailButton = (taskName)
                    ? `<button class="header-btn btn-header-action" onclick="location.hash = 'history-detail&task=${taskName}&type=로봇호출'">주문 내역</button>`
                    : placeholder;

                pageContent = `
                    <div class="box-header">
                        ${placeholder}
                        <h2 class="page-title"></h2>
                        ${historyDetailButton}
                    </div>
                    <div class="status-section vertical-center">
                        <h1 class="status-text" style="font-size: 28px; font-weight: 600;">호출 요청이<br>접수되었습니다.</h1>
                        <img src="./assets/images/robot_icon.png" alt="robot" class="robot-icon-large"/>
                        <p class="estimated-time-text">예상 대기 시간 ${waitTime || '??'}분</p>
                    </div>
                    <button class="btn-bottom-close" onclick="location.hash='/'">닫기</button>
                `;
                break;
            } // ⬅️ 닫는 중괄호 추가
            
            case "order-success": { // ⬅️ 여는 중괄호 추가
                const orderTaskName = params.get('task');
                const taskType = params.get('type'); 
                
                const historyDetailButton = (orderTaskName && taskType)
                    ? `<button class="header-btn btn-header-action" onclick="location.hash = 'history-detail&task=${orderTaskName}&type=${taskType}'">주문 내역</button>`
                    : placeholder;
                
                pageContent = `
                    <div class="box-header">
                        ${placeholder}
                        <h2 class="page-title">요청 완료</h2>
                        ${historyDetailButton}
                    </div>
                    <div class="status-section vertical-center">
                        <h1 class="status-text">주문 요청이<br>접수되었습니다.</h1>
                        <p class="task-name-display">주문 번호 : ${orderTaskName}</p>
                        <img src="./assets/images/robot_icon.png" alt="robot" class="robot-icon-large"/>
                        <p class="estimated-time-text">예상 대기 시간 20분</p>
                    </div>
                    <button class="btn-bottom-close" onclick="location.hash='/'">닫기</button>
                `;
                break;
            } // ⬅️ 닫는 중괄호 추가

            case "menu":
                pageContent = `
                    <div class="box-header">
                        ${backButton}
                        <h2 class="page-title">${currentOrderType === 'food' ? '음식 주문' : '비품 요청'}</h2>
                        ${placeholder}
                    </div>
                    <div class="content-area" id="menu-list-container"></div>
                    <footer class="footer">
                        <span class="total-price" id="total_price_footer">총 0원</span>
                        <div class="divider"></div>
                        <button id="button_cart" class="cart-button" onclick="location.hash='cart'">장바구니 보기</button>
                    </footer>`;
                break;

            case "cart":
                const isFood = currentOrderType === 'food';
                pageContent = `
                    <div class="box-header">
                        ${backButton}
                        <h2 class="page-title">주문상품</h2>
                        ${placeholder}
                    </div>
                    <div class="content-area" id="cart-list"></div>
                    <div class="footer-cart">
                         ${isFood ? `<div class="total-price-container"><span>상품금액</span><span id="total-price-display">0원</span></div>` : `<div class="notice">비품 요청은 비용이 청구되지 않습니다.</div>`}
                        <button id="btn-order" class="order-button">${isFood ? '주문하기' : '요청하기'}</button>
                    </div>`;
                break;

            case "history":
                pageContent = `
                    <div class="box-header">
                        ${backButton}
                        <h2 class="page-title">요청 이력</h2>
                        ${placeholder}
                    </div>
                    <div id="history-result-container" class="history-items"></div>`;
                break;
                
            case "history-detail":
                pageContent = `
                    <div class="box-header">
                        ${backButton}
                        <h2 class="page-title">상세 정보</h2>
                        ${placeholder}
                    </div>
                    <main id="history-detail-container" class="content"></main>`;
                break;

            default:
                pageContent = `<p>페이지를 찾을 수 없습니다.</p>`;
                break;
        }

        template = `
            <header class="sub-header">
                <span class="logo">ROOMIE</span>
                <span class="room-number">${ROOM_ID}</span>
            </header>
            <div class="sub-page-box">
                ${pageContent}
            </div>
        `;
    }

    // --- 최종 렌더링 및 후속 작업 ---
    container.innerHTML = template
    addEventListenersForRoute(mainRoute);

    if (mainRoute === 'menu') {
        order.loadAndRenderMenu(currentOrderType, 'menu-list-container');
    } else if (mainRoute === 'cart') {
        order.renderCartContents('cart-list', currentOrderType);
    } else if (mainRoute === 'history') {
        history.loadHistoryList("history-result-container");
    } else if (mainRoute === 'history-detail') {
        history.renderHistoryDetail("history-detail-container");
    }
}

function addEventListenersForRoute(route) {
    if (route === 'cart') {
        const orderButton = document.getElementById('btn-order');
        if (orderButton) {
            orderButton.addEventListener('click', () => {
                orderButton.disabled = true;
                orderButton.textContent = '주문 처리 중...';
                order.createOrder(currentOrderType);
            });
        }
    }
}