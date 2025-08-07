// common.js app.js와 중복되는 초기화 로직을 제거하고, 순수한 유틸리티와 WebSocket 모듈
// API 요청을 보내는 범용 함수
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
let toastTimer; // 타이머 ID를 저장할 변수

export function showToast(imageSrc, message = "") { // 두 번째 인자로 메시지를 받도록 수정
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
    toastMessage.textContent = message; // 메시지 텍스트 설정
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
            const data = JSON.parse(event.data);
            if (data.type !== "event") return;

            // ✅ "호출 수락" 이벤트를 받으면 페이지를 직접 전환합니다.
            if (data.action === "call_request_acceptance") {
                const taskName = data.payload.task_name;
                const waitTime = data.payload.estimated_wait_time;
                
                // URL에 작업명(task)과 대기시간(wait)을 담아 전달
                location.hash = `robot-accepted&task=${taskName}&wait=${waitTime}`;
                return; // 페이지를 전환했으므로 아래 로직은 실행하지 않음
            }

            // --- 다른 이벤트에 대한 토스트 알림 (기존 로직 유지) ---
            let imagePath = "";
            let message = ""; 
            
            switch (data.action) {
                case "delivery_completion":
                    imagePath = "../assets/images/delivery_completed_notification.png";
                    break;
                case "task_timeout_return":
                    imagePath = "../assets/images/timeout_return_notification.png"
                    break;
                // 'robot_arrival_completion' 등 다른 이벤트에 대한 처리 추가 가능
                default:
                    console.warn("알 수 없는 이벤트 액션:", data.action);
                    return;
            }
            
            if (imagePath) {
                showToast(imagePath, message);
            }

        } catch (error) {
            console.error("WebSocket 메시지 처리 중 오류 발생:", error);
        }
    };

    ws.onclose = () => {
        console.log("WebSocket 연결이 종료되었습니다.");
    };

    ws.onerror = (error) => {
        console.error("WebSocket 오류 발생:", error);
    };
}


// WebSocket 메시지 처리 로직
function handleWebSocketMessage(event) {
    try {
        const data = JSON.parse(event.data);
        console.log("WebSocket 메시지 수신:", data);

        if (data.type === "event") {
            switch (data.action) {
                case "task_timeout_return":
                    showToast('../assets/images/timeout_return_notification.png');
                    break;
                case "delivery_completion":
                    showToast('../assets/images/delivery_completed_notification.png');
                    // 필요 시, 특정 주문 상세 페이지로 이동하거나 상태를 업데이트하는 로직 추가
                    break;
                case "robot_arrival_completion":
                    showToast('../assets/images/robot_arrived_pickup.png');
                    location.hash = 'robot-success'; // 로봇 도착 시 성공 화면으로 자동 전환
                    break;
                case "call_request_acceptance":
                    showToast('../assets/images/call_accepted.png');
                    break;
                default:
                    console.warn("알 수 없는 이벤트 액션:", data.action);
            }
        }
    } catch (e) {
        console.error("WebSocket 메시지 파싱 오류:", e);
    }
}