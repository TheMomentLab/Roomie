// common.js app.js와 중복되는 초기화 로직을 제거하고, 순수한 유틸리티와 WebSocket 모듈
// API 요청을 보내는 범용 함수

let toastTimer; // 타이머 ID를 저장할 변수

export function sendApiRequest(url, data) {
  return fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(data),
  }).then((res) => res.json());
}

/**
 * 이미지 경로를 받아 화면에 알림창(토스트)을 표시하는 함수
 * @param {string} imageSrc - 표시할 이미지의 경로
 */


export function showToast(imageSrc) { // 두 번째 인자로 메시지를 받도록 수정
    const globalToast = document.getElementById('global-toast');
    const toastImage = document.getElementById('toast-full-image');
    const toastMessage = document.getElementById('toast-message'); // 메시지를 표시할 요소 추가 (HTML에도 추가 필요)

    if (!globalToast || !toastImage || !toastMessage) {
        console.error("알림창을 위한 HTML 요소를 찾을 수 없습니다.");
        return;
    }

    if (toastTimer) {
        clearTimeout(toastTimer);
    }

    toastImage.src = imageSrc;
    globalToast.classList.remove('hidden');

    toastTimer = setTimeout(() => {
        globalToast.classList.add('hidden');
    }, 2000);
}


// --- 웹소켓 초기화 함수 (이미지 경로만 전달하도록 수정) ---
export function initWebSocket() {
    const ws = new WebSocket(window.WS_URL);

    ws.onopen = () => {
        console.log("WebSocket 연결 성공");
    };

    ws.onmessage = (event) => {
        try {
            const eventData = JSON.parse(event.data);
            if (eventData.type !== "event") return; // 이벤트가 아니면 무시

            // API 명세서(message.md)와 app.js의 라우팅 로직을 기반으로 수정
            switch (eventData.action) {
                case "call_request_acceptance":
                    // 호출 수락 시, '로봇 호출 접수' 페이지로 이동
                    const { task_name, estimated_wait_time } = eventData.payload;
                    location.hash = `robot-accepted&task=${task_name}&wait=${estimated_wait_time}`;
                    break;

                case "robot_arrival_completion":
                    // 로봇 도착 시, 알림 표시
                    showToast("../assets/images/robot_arrived_pickup.png");
                    // 필요 시, 특정 페이지로 강제 이동시킬 수 있습니다.
                    // 예: if(location.hash.includes('history-detail')) location.hash = 'robot-success';
                    break;
                    
                case "delivery_completion":
                    // 배송 완료 시, 알림 표시
                    showToast("../assets/images/delivery_completed_notification.png");
                    break;

                case "task_timeout_return":
                    // 타임아웃 시, 알림 표시
                    showToast("../assets/images/timeout_return_notification.png");
                    break;

                default:
                    console.warn("알 수 없는 WebSocket 이벤트 액션:", eventData.action);
                    return; 
            }

        } catch (error) {
            console.error("WebSocket 메시지 처리 중 오류 발생:", error);
        }
    };

    ws.onclose = () => {
        console.log("WebSocket 연결이 종료되었습니다. 5초 후 재연결을 시도합니다.");
        // 안정성을 위해 재연결 로직 추가를 고려할 수 있습니다.
        // setTimeout(initWebSocket, 5000);
    };

    ws.onerror = (error) => {
        console.error("WebSocket 오류 발생:", error);
    };
}
