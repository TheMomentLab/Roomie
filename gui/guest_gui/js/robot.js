// js/robot.js

/**
 * 로봇 호출 처리 모듈
 * - 호출 요청 후 selectedTask에 작업 정보 저장
 * - 상태 페이지로 이동 (타임라인은 history.js에서 처리)
 */

export async function callRobot(destination) {
  const request = {
    type: "request",
    action: "call_robot_task",
    payload: {
      location_name: ROOM_ID,
      task_type_name: "로봇호출",
      robot_type_name: "서빙로봇",
      destination_name: destination || ROOM_ID
    }
  };

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
      window.location.href = `index.html#status-history&task=${result.payload.task_name}`;
    } else {
      showToast("asset/error_toast.png", "error");
    }
  } catch (err) {
    console.error("로봇 호출 실패:", err);
    showToast("asset/error_toast.png", "error");
  }
}
