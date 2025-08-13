import { sendApiRequest, showToast } from './common.js'; // ⬅️ 1. showToast 임포트 추가


// --- 시간 포맷 함수 (공통 사용) ---
function formatTime(isoString) {
    if (!isoString) return '';
    const date = new Date(isoString);
    let hours = date.getHours();
    const minutes = date.getMinutes().toString().padStart(2, '0');
    const ampm = hours >= 12 ? '오후' : '오전';
    hours = hours % 12;
    hours = hours ? hours : 12;
    return `${ampm} ${hours}:${minutes}`;
}

let historyInterval; // 주기적 업데이트를 제어하기 위한 변수


export async function loadHistoryList(containerId) { // ⬅️ containerId 인자 추가
  try {
    const request = {
      type: "request",
      action: "get_task_list",
      payload: {
        request_location: window.ROOM_ID
      }
    };
    
    // ⬅️ 3. API 전체 주소를 사용하도록 수정
    const result = await sendApiRequest(`${window.API_URL}/get_task_list`, request);

    if (result && result.payload && result.payload.order_details?.tasks) {
      localStorage.setItem("orderHistory", JSON.stringify(result.payload.order_details.tasks));
      renderHistoryList(containerId, result.payload.order_details.tasks);
    } else {
      showToast("assets/error_toast.png", "error");
    }
  } catch (err) {
    console.error("이력 조회 실패:", err);
    showToast("assets/error_toast.png", "error");
  }
}

function renderHistoryList(containerId, list) {
  const container = document.getElementById(containerId);
  if (!container) return;

  if (list.length === 0) {
    container.innerHTML = "<p style='text-align:center; color:#888;'>요청 이력이 없습니다.</p>";
    return;
  }
  
  // ⬅️ 4. CSS와 일치하도록 클래스 이름 수정
  container.innerHTML = list.map(item => `
    <div class="task-card" data-task-name="${item.task_name}" data-task-type="${item.task_type_name}">
      <strong class="task-title">${item.task_type_name}</strong>
      <span class="task-time">${formatTime(item.created_at)}</span>
      <button class="btn-detail">상세보기</button>
    </div>
  `).join("");

  container.addEventListener('click', (e) => {
    if (e.target.classList.contains('btn-detail')) {
      const historyItem = e.target.closest('.task-card');
      const taskName = historyItem.dataset.taskName;
      const taskType = historyItem.dataset.taskType;
      
      location.hash = `history-detail&task=${taskName}&type=${taskType}`;
    }
  });
}
export async function renderHistoryDetail(containerId) {
    if (historyInterval) clearInterval(historyInterval);

    const container = document.getElementById(containerId);
    if (!container) {
        console.error(`[History] 컨테이너를 찾을 수 없습니다: #${containerId}`);
        return;
    }

    
    // ✅ 상세 페이지 템플릿을 컨테이너 내부에 먼저 렌더링
    container.innerHTML = `
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
    `;
    
      // URL에서 taskName과 taskType 추출
    const route = location.hash.replace("#", "");
    const mainRoute = route.split('&')[0]; // 'history-detail'
    const queryString = route.includes('&') ? route.substring(route.indexOf('&') + 1) : '';
    const params = new URLSearchParams(queryString);
    const taskName = params.get('task');
    const taskType = params.get('type');

    if (!taskName || !taskType) {
        container.innerHTML = `<p>조회할 작업 정보가 없습니다. URL을 확인해주세요.</p>`;
        return;
    }

    // 실제 API 호출 및 UI 업데이트를 수행하는 함수
    async function fetchAndUpdateStatus() {
        try {
            const isCall = taskType.includes("호출");
            const url = isCall ? `${window.API_URL}/get_call_history` : `${window.API_URL}/get_order_history`;
            const action = isCall ? "get_call_history" : "get_order_history";

            let payload;
            if (isCall) {
                payload = { location_name: window.ROOM_ID, task_name: taskName };
            } else {
                payload = { request_location: window.ROOM_ID, task_name: taskName, task_type_name: taskType };
            }
            const request = { type: "request", action, payload };

            const result = await sendApiRequest(url, request);
            updateUI(result.payload, isCall);

        } catch (err) {
            console.error("상세 이력 업데이트 실패:", err);
            // 에러 발생 시, 주기적 업데이트 중지
            if (historyInterval) clearInterval(historyInterval);
        }
    }

    // UI를 업데이트하는 함수
      function updateUI(p, isCall) {
        if (!p) return;

        const taskTypeNameEl = container.querySelector('#task_type_name');
        const locationNameEl = container.querySelector('#request_location_name');
        const estimatedTimeEl = container.querySelector('#estimated_time');
        const timelineContainerEl = container.querySelector('#status-timeline');

        if (taskTypeNameEl) taskTypeNameEl.textContent = isCall ? "로봇 호출" : p.task_type_name;
        if (locationNameEl) locationNameEl.textContent = `목적지 ${p.request_location || p.location_name}`;
        
        // ✅ [수정] '호출' 타입과 '주문' 타입을 분기하여 처리합니다.
        if (isCall) {
            // --- 호출(Call) 타입 처리 ---
            // API가 제공하는 예상 시간을 직접 표시합니다.
            if (estimatedTimeEl) estimatedTimeEl.textContent = `약 ${p.estimated_time}분 후 도착 예정`;

            // '호출 접수' 타임라인 아이템을 표시하되, 시간 정보는 API에 없으므로 공백으로 둡니다.
            if (timelineContainerEl) {
                timelineContainerEl.innerHTML = `
                    <div class="timeline-item">
                        <div class="timeline-point"><div class="point"></div><div class="line"></div></div>
                        <div class="details"><span class="label">호출 접수됨</span><span class="time"></span></div>
                    </div>
                    <div class="timeline-item inactive">
                        <div class="timeline-point"><div class="point"></div><div class="line"></div></div>
                        <div class="details"><span class="label">로봇 도착</span><span class="time"></span></div>
                    </div>
                `;
            }

        } else {
            // --- 주문(Order) 타입 처리 (기존 로직 유지) ---
            const timelineData = [
                { id: 'status_creation', time: p.task_creation_time, text: '주문 접수됨' },
                { id: 'status_assignment', time: p.robot_assignment_time, text: '로봇 배정됨' },
                { id: 'status_pickup', time: p.pickup_completion_time, text: '픽업 완료됨' },
                { id: 'status_arrival', time: p.delivery_arrival_time, text: '배달 완료됨' }
            ];
            
            const lastCompleted = [...timelineData].reverse().find(s => s.time);
            if (lastCompleted && lastCompleted.id !== 'status_arrival' && p.estimated_time) {
                const arrivalTime = new Date(new Date(lastCompleted.time).getTime() + p.estimated_time * 1000);
                if (estimatedTimeEl) estimatedTimeEl.textContent = `${formatTime(arrivalTime.toISOString())} 도착 예정`;
            } else {
                if (estimatedTimeEl) estimatedTimeEl.textContent = '완료';
            }

            if (timelineContainerEl) {
                timelineContainerEl.innerHTML = ''; 
                timelineData.forEach(status => {
                    const item = document.createElement('div');
                    item.className = `timeline-item ${!status.time ? 'inactive' : ''}`;
                    item.innerHTML = `
                        <div class="timeline-point"><div class="point"></div><div class="line"></div></div>
                        <div class="details">
                            <span class="label">${status.text}</span>
                            <span class="time">${formatTime(status.time)}</span>
                        </div>`;
                    timelineContainerEl.appendChild(item);
                });
            }
        }
    }

    // 최초 1회 즉시 실행 후, 15초마다 주기적으로 상태 업데이트
    fetchAndUpdateStatus();
    historyInterval = setInterval(fetchAndUpdateStatus, 15000);
}