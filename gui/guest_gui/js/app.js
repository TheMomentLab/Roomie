// app.js

import * as order from "./order.js";
import * as robot from "./robot.js";
import * as history from "./history.js";
import { initWebSocket } from "./common.js";

// --- 전역 설정 ---
function getConfigFromRoomParam() {
<<<<<<< HEAD
  const roomParam = new URLSearchParams(window.location.search).get("room") || "ROOM_201";
  const configMap = {
    ROOM_201: { ROOM_ID: "ROOM_201", ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
=======
  const roomParam = new URLSearchParams(window.location.search).get("room") || "ROOM_101";
  const configMap = {
    ROOM_201: { ROOM_ID: "ROOM_201", ENABLED_FEATURES: { food: true, supply: true, robot: true, history: true } },
    ROOM_101: { ROOM_ID: "ROOM_101", ENABLED_FEATURES: { food: true, supply: false, robot: true, history: true } },
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
    ROOM_102: { ROOM_ID: "ROOM_102", ENABLED_FEATURES: { food: true, supply: false, robot: true, history: true } },
    LOB_CALL: { ROOM_ID: "LOB_CALL", ENABLED_FEATURES: { food: false, supply: false, robot: true, history: true } },
    RES_CALL: { ROOM_ID: "RES_CALL", ENABLED_FEATURES: { food: true, supply: false, robot: true, history: false } }
  };
<<<<<<< HEAD
  return configMap[roomParam] || configMap["ROOM_201"];
=======
  return configMap[roomParam] || configMap["ROOM_101"];
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
}


// index.html폴더에서 실행
// python3 -m http.server 8000        

// 자기 ip로 수정
// http://192.168.0.8:8000/?room=ROOM_102 
const CONFIG = getConfigFromRoomParam();
const ROOM_ID = CONFIG.ROOM_ID;
const ENABLED_FEATURES = CONFIG.ENABLED_FEATURES;
window.ROOM_ID = ROOM_ID;
const IS_DEV = true;
<<<<<<< HEAD
const BASE_URL = IS_DEV ? "http://localhost:8072" : "https://production-url.com";
=======
const BASE_URL = IS_DEV ? "http://0.0.0.0:8072" : "https://roomie.com";
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
window.API_URL = `${BASE_URL}/api/gui`;
window.WS_URL = `${BASE_URL.replace("http", "ws")}/ws/guest/${ROOM_ID}`;

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
<<<<<<< HEAD
    const hash = location.hash.replace("#", "").split("&")[0] || "/";
    renderPageTemplate(hash);
}


=======
    const hash = location.hash.replace("#", "") || "/";
    renderPageTemplate(hash);
}
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
function renderPageTemplate(route) {
    const container = document.querySelector(".main-content");
    if (!container) return;
    
<<<<<<< HEAD
    // 1. 메인 헤더와 서브 페이지 헤더 DOM 요소를 미리 선택
    const mainHeader = document.querySelector('.header'); 
    let template = '';
    const backButton = `<div class="back-button" onclick="history.back()"></div>`;
    const homeButton = `<div class="back-button" onclick="location.hash='/'"></div>`;
    // 2. subPageHeader에서 ROOMIE 로고가 잘 보이도록 별도 클래스 추가
    const subPageHeader = `
        <header class="sub-header">
            <span class="logo">ROOMIE</span>
            <span class="room-number">${ROOM_ID}</span>
        </header>`;

    // 3. 라우트 조건에 따라 클래스와 헤더 표시 여부 제어
    if (route === '/') {
        container.className = 'main-content main-content-home';
        if(mainHeader) mainHeader.style.display = 'block'; // 메인 화면에서는 메인 헤더 보이기
    } else if (route === 'call-robot-loading') {
        container.className = 'main-content main-content-full';
        if(mainHeader) mainHeader.style.display = 'none'; // 로딩 중에는 모든 헤더 숨기기
    } else {
        container.className = 'main-content sub-page-box'; // 나머지 모든 페이지는 흰색 박스 스타일 적용
        if(mainHeader) mainHeader.style.display = 'none'; // 서브 페이지에서는 메인 헤더 숨기기
    }

    switch (route) {
        case "/":
            // index.html의 기존 헤더를 사용하므로 템플릿에서 헤더 제거
            template = `
                <h1>ROOMIE</h1>
                <p class="subtitle">호텔 안내 & 주문 배송 서비스</p>
                <p class="welcome-message">ROOMIE에 오신 걸 환영합니다.</p>
                <div class="button-grid">
                    <button id="btn-call-robot" class="grid-button">로봇호출</button>
                    <button id="btn-order-history" class="grid-button">요청조회</button>
                    <button id="btn-order-food" class="grid-button">음식주문</button>
                    <button id="btn-order-supply" class="grid-button">비품주문</button>
                </div>`;
            break;
        case "menu":
            template = `
                ${subPageHeader}
                ${backButton}
                <div class="menu-list" id="menu-list-container"></div>
                <footer class="footer">
                    <span class="total-price" id="total_price_footer">총 0원</span>
                    <div class="divider"></div>
                    <button id="button_cart" class="cart-button" onclick="location.hash='cart'">장바구니 보기</button>
                </footer>`;
            break;
        case "cart":
            const isFood = currentOrderType === 'food';
            template = `
                ${subPageHeader}
                <div class="cart-header">
                    ${backButton}
                    <h2 class="cart-title">주문상품</h2>
                </div>
                <div id="cart-list" class="cart-items"></div>
                <div class="footer-cart">
                    ${isFood ? `<div class="total-price-container"><span>상품금액</span><span id="total-price-display">0원</span></div>` : `<div class="notice">비품 요청은 비용이 청구되지 않습니다.</div>`}
                    <button id="btn-order" class="order-button">${isFood ? '주문하기' : '요청하기'}</button>
                </div>`;
            break;
        case "order-success":
        case "robot-success":
            // ✅ 예상 시간 정보를 localStorage에서 가져옴
            const estimatedTime = localStorage.getItem("estimatedTime");
            const timeText = estimatedTime ? `예상 도착 시간: 약 ${estimatedTime}분 후` : '요청 내역을 확인해주세요.';
            localStorage.removeItem("estimatedTime"); // 한번 사용 후 삭제

            template = `
                ${subPageHeader}
                ${homeButton}
                <div class="status-section">
                    <h1 class="status-text">요청이<br>접수되었습니다.</h1>
                    <p class="estimated-time-text">${timeText}</p>
                    <img src="./assets/images/robot_icon.png" alt="robot" class="robot-icon"/>
                    <button class="order-history-button" onclick="location.hash = 'history-detail'">요청 내역 확인</button>
                </div>`;
            break;
        case "call-robot-loading":
             template = `
                <span class="loading-message">호출 가능한 로봇이<br>있는지 확인중입니다...</span>
                <button class="cancel-button" onclick="history.back()">취소</button>`;
            break;
        case "history":
             template = `
                ${subPageHeader}
                ${homeButton}
                <div class="cart-header"><h2 class="cart-title">요청 이력</h2></div>
                <div id="history-result" class="history-items"></div>`;
            break;
        case "history-detail":
            template = `
                ${subPageHeader}
                ${backButton}
                <main class="content">
                    <div class="status-card">
                        <div class="status-header">
                            <h2 id="task_type_name"></h2>
                            <div class="status-summary">
                                <p id="estimated_time"></p>
                                <p id="request_location_name"></p>
                            </div>
                        </div>
                        <div id="status-timeline" class="status-timeline"></div>
                    </div>
                </main>`;
            break;
        default:
            template = `<p>페이지를 찾을 수 없습니다.</p>`;
            break;
    }

    container.innerHTML = template;

    // 템플릿 렌더링 후, 데이터 로딩 및 이벤트 리스너 추가
    addEventListenersForRoute(route);

    if (route === 'menu') {
        order.loadAndRenderMenu(currentOrderType, 'menu-list-container');
    } else if (route === 'cart') {
        order.renderCartContents('cart-list', currentOrderType);
    } else if (route === 'history') {
        history.loadHistoryList("history-result");
    } else if (route === 'history-detail') {
=======
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

        template = `
            <h1>ROOMIE</h1>
            <p class="subtitle">호텔 안내 & 주문 배송 서비스</p>
            <p class="welcome-message">ROOMIE에 오신 걸 환영합니다.</p>
            <div class="button-grid">
                <button id="btn-call-robot" class="grid-button">로봇호출</button>
                <button id="btn-order-history" class="grid-button">요청조회</button>
                <button id="btn-order-food" class="grid-button">음식주문</button>
                <button id="btn-order-supply" class="grid-button">비품주문</button>
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

            case "robot-accepted":
                const taskName = params.get('task');
                const waitTime = params.get('wait');
                pageContent = `
                    <div class="box-header">
                        ${placeholder}
                        <h2 class="page-title"></h2>
                        <button class="header-btn btn-header-action" onclick="location.hash='history'">주문 내역</button>
                    </div>
                    <div class="status-section vertical-center">
                        <h1 class="status-text" style="font-size: 28px; font-weight: 600;">호출 요청이<br>접수되었습니다.</h1>
                        <img src="./assets/images/robot_icon.png" alt="robot" class="robot-icon-large"/>
                        <p class="estimated-time-text">예상 대기 시간 ${waitTime || '??'}분</p>
                    </div>
                    <button class="btn-bottom-close" onclick="location.hash='/'">닫기</button>
                `;
                break;
            
            case "order-success":
                const orderTaskName = params.get('task');
                const taskType = params.get('type'); 
                const historyDetailButton = (orderTaskName && taskType)
                    // =================================================================
                    // ✅ 2. 주문 내역 버튼에 새 클래스 적용
                    // =================================================================
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
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
        history.renderHistoryDetail("history-detail-container");
    }
}

<<<<<<< HEAD
// ✅ 3. 중복된 함수 선언 제거 (하나로 통합)
=======
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
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
<<<<<<< HEAD
}
=======
}
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
