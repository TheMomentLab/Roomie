// history.js
// 요청 이력 조회(get_task_list) 및 단건 상태 상세 확인(get_order_detail, get_order_history, get_call_history) 처리

import { sendApiRequest } from './common.js';

/**
 * 요청 이력 조회 요청 타입
 * @typedef {Object} GetTaskListRequest
 * @property {string} type - "request"
 * @property {string} action - "get_task_list"
 * @property {Object} payload
 * @property {string} payload.request_location - 위치 이름(예: ROOM_201)
 */

/**
 * 요청 이력 조회 응답 타입
 * @typedef {Object} GetTaskListResponse
 * @property {string} type - "response"
 * @property {string} action - "get_task_list"
 * @property {Object} payload
 * @property {string} payload.location_name
 * @property {Object} payload.order_details
 * @property {Array<Object>} payload.order_details.tasks - 요청 이력 배열
 * @property {string} payload.order_details.tasks[].task_name - 작업 ID
 * @property {string} payload.order_details.tasks[].task_type_name - 작업 유형명
 * @property {string} payload.order_details.tasks[].created_at - 생성일시 (ISO8601)
 */

export async function loadHistoryList() {
  try {
    /** @type {GetTaskListRequest} */
    const request = {
      type: "request",
      action: "get_task_list",
      payload: {
        request_location: ROOM_ID
      }
    };

    /** @type {GetTaskListResponse['payload']} */
    const result = await sendApiRequest("/api/gui/get_task_list", request);

    if (result && result.order_details?.tasks) {
      localStorage.setItem("orderHistory", JSON.stringify(result.order_details.tasks));
      renderHistoryList("history-result", result.order_details.tasks);
    } else {
      showToast("asset/error_toast.png", "error");
    }
  } catch (err) {
    console.error("이력 조회 실패:", err);
    showToast("asset/error_toast.png", "error");
  }
}

function renderHistoryList(containerId, list) {
  const container = document.getElementById(containerId);
  if (!container) return;

  if (list.length === 0) {
    container.innerHTML = "<p style='color:#888;'>요청 이력이 없습니다.</p>";
    return;
  }

  container.innerHTML = list.map(item => `
    <div class="history-item" data-task="${item.task_name}">
      <strong>${item.task_type_name}</strong>
      <span>${formatTime(item.created_at)}</span>
      <button class="btn-detail">상세보기</button>
    </div>
  `).join("");

  container.querySelectorAll(".btn-detail").forEach((btn, i) => {
    btn.addEventListener("click", () => {
      const taskName = list[i].task_name;
      const taskType = list[i].task_type_name;
      localStorage.setItem("selectedTask", taskName);
      localStorage.setItem("selectedTaskType", taskType);
      window.location.hash = `status-history&task=${taskName}`;
    });
  });
}

function formatTime(isoStr) {
  if (!isoStr) return "";
  const d = new Date(isoStr);
  const h = d.getHours();
  const m = d.getMinutes().toString().padStart(2, '0');
  const ampm = h >= 12 ? '오후' : '오전';
  const hour = h % 12 || 12;
  return `${ampm} ${hour}:${m}`;
}

export async function renderHistoryDetail(containerId) {
  const container = document.getElementById(containerId);
  const taskName = localStorage.getItem("selectedTask");
  const taskType = localStorage.getItem("selectedTaskType");
  if (!container || !taskName || !taskType) return;

  const isCall = taskType.includes("호출");
  const url = isCall ? "/api/gui/get_call_history" : "/api/gui/get_order_history";
  const action = isCall ? "get_call_history" : "get_order_history";

  try {
    const request = {
      type: "request",
      action,
      payload: {
        request_location: ROOM_ID,
        task_name: taskName,
        task_type_name: taskType
      }
    };

    const result = await sendApiRequest(url, request);
    if (!result || !result.payload) {
      container.innerHTML = "<p style='color:#888;'>상세 정보를 불러올 수 없습니다.</p>";
      return;
    }

    const p = result.payload;
    const timeline = isCall ? [
      { label: "호출 생성", key: "task_creation_time" },
      { label: "로봇 출발", key: "robot_assignment_time" },
      { label: "로봇 도착", key: "robot_arrival_time" }
    ] : [
      { label: "주문 접수", key: "task_creation_time" },
      { label: "로봇 할당", key: "robot_assignment_time" },
      { label: "픽업 완료", key: "pickup_completion_time" },
      { label: "도착 완료", key: "delivery_arrival_time" }
    ];

    container.innerHTML = `
      <h2>${taskType} 상세 상태</h2>
      <p><strong>${taskName}</strong> - ${p.request_location}</p>
      <div class="timeline">
        ${timeline.map(t => `
          <div class="timeline-item ${!p[t.key] ? 'inactive' : ''}">
            <div class="point"></div>
            <div class="details">
              <span class="label">${t.label}</span>
              <span class="time">${formatTime(p[t.key])}</span>
            </div>
          </div>
        `).join("")}
      </div>
    `;
  } catch (err) {
    console.error("상세 이력 불러오기 실패:", err);
    container.innerHTML = "<p style='color:#888;'>오류가 발생했습니다.</p>";
  }
}

/*
📌 시나리오 흐름 (Mermaid.js)

```mermaid
sequenceDiagram
  participant Guest
  participant GUI
  participant Server

  Guest->>GUI: 요청 이력 조회 클릭
  GUI->>Server: POST /api/gui/get_task_list
  Server-->>GUI: tasks[] 응답
  GUI->>Guest: 리스트 렌더링

  Guest->>GUI: 상세보기 클릭
  GUI->>Server: POST /api/gui/get_order_history 또는 get_call_history
  Server-->>GUI: payload 응답
  GUI->>Guest: 상세 타임라인 렌더링
```
*/
