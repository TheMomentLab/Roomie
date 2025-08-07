// js/robot.js

/**
 * 로봇 호출 처리 모듈
 * - 호출 요청 후 selectedTask에 작업 정보 저장
 * - 상태 페이지로 이동 (타임라인은 history.js에서 처리)
 */


export function handleWebSocketEvent(event) {
  const { action, payload } = event;

  switch (action) {
    case "call_request_acceptance":
      showToast(`호출이 수락되었습니다. 예상 대기시간: ${payload.estimated_wait_time}분`);
      break;

    case "robot_arrival_completion":
      renderCallReadyView();
      break;

    case "task_timeout_return":
      showToast(`시간초과로 로봇이 복귀했습니다.`);
      break;

    default:
      console.log("Unhandled WS event:", action);
  }
}


export async function callRobot(destination) {
  const request = {
    type: "request",
    action: "create_call_task",
    payload: {
      location_name: ROOM_ID,
      task_type_id: 2,
      destination_name: destination || ROOM_ID
    }
  };

  renderRobotCalling(); // 호출 중 화면 표시
  try {
    const response = await fetch(`${API_URL}/call_robot_task`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request)
    });

    const result = await response.json();
    
    if (result.type === "response" && result.payload.success) {
      // ✅ 가장 최근 작업 정보만 저장
      localStorage.setItem("selectedTask", result.payload.task_name);
      localStorage.setItem("selectedTaskType", "로봇호출");
     location.hash = `call-ready&task=${result.payload.task_name}`;
    } 
    else 
    {
      showToast("assets/error.png", result.payload.error_message || "호출 실패");
      location.hash = "home";
    }
  } 
  catch (err) {
    console.error("로봇 호출 실패:", err);
    showToast("asset/error_toast.png", "error");
  }
}

export function renderRobotCalling() {
  const container = document.getElementById("main-ui");
  container.innerHTML = `
    <div class="view-call-robot app-layout">
      <span class="loading-message">호출 가능한 로봇이\n있는지 확인중입니다.</span>
    </div>
  `;
}

// 로봇 호출 중 화면 렌더링
export function renderRobotWaitingView() {
  const app = document.getElementById('app');
  app.innerHTML = `
    <div class="app-layout view-call-robot">
      <div class="loading-message">
        로봇을 호출 중입니다...<br/>잠시만 기다려 주세요.
      </div>
    </div>
  `;
}

// 로봇 도착 완료 화면 렌더링
export function renderCallReadyView() {
  const app = document.getElementById('app');
  app.innerHTML = `
    <div class="app-layout view-call-ready">
      <div class="status-section">
        <h2>로봇이 도착했습니다!</h2>
        <div class="timeline">
          <div class="timeline-step">호출 접수</div>
          <div class="timeline-step">로봇 이동 중</div>
          <div class="timeline-step">도착 완료</div>
        </div>
        <button class="close-button">닫기</button>
      </div>
    </div>
  `;

  document.querySelector('.close-button').addEventListener('click', () => {
    window.location.hash = '/';
  });
}
