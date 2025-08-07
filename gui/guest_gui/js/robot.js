<<<<<<< HEAD
// js/robot.js (수정 후)
import { showToast } from './common.js';

export async function callRobot() {
  // 1. 먼저 '호출 중' 화면으로 전환
=======
import { showToast } from './common.js';

export async function callRobot() {
  // 1. 먼저 '호출 중' 로딩 화면으로 전환
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
  location.hash = 'call-robot-loading';

  const request = {
    type: "request",
    action: "create_call_task",
    payload: {
      location_name: window.ROOM_ID,
<<<<<<< HEAD
      task_type_id: 2 // API 명세에 따른 '호출' 타입
=======
      task_type_id: 2 // '호출' 타입
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
    }
  };

  try {
<<<<<<< HEAD
    // 2. API 명세에 맞는 올바른 주소로 API 호출
=======
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
    const response = await fetch(`${window.API_URL}/create_call_task`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(request)
    });

    const result = await response.json();
    
<<<<<<< HEAD
    if (result.payload && result.payload.success) {
      // 3. 성공 시, 작업 정보를 저장하고 '호출 성공' 화면으로 전환
      localStorage.setItem("selectedTask", result.payload.task_name);
      localStorage.setItem("selectedTaskType", "로봇호출");
      location.hash = `robot-success`;
    } else {
      showToast("./assets/images/error_toast.png", result.payload.error_message || "호출에 실패했습니다.");
      location.hash = "/"; // 실패 시 메인 화면으로 복귀
    }
  } catch (err) {
    console.error("로봇 호출 API 요청 실패:", err);
    showToast("./assets/images/error_toast.png", "오류가 발생했습니다.");
    location.hash = "/"; // 실패 시 메인 화면으로 복귀
=======
    // 2. ✅ API 요청이 성공해도 화면을 전환하지 않고 대기합니다.
    //    실패했을 때만 메인 화면으로 돌려보냅니다.
    if (!result.payload || !result.payload.success) {
      showToast("./assets/images/error_toast.png", result.payload.error_message || "호출에 실패했습니다.");
      location.hash = "/"; // 실패 시 메인 화면으로 복귀
    }
    // 성공 시에는 아무것도 하지 않고 웹소켓 이벤트를 기다립니다.

  } catch (err) {
    console.error("로봇 호출 API 요청 실패:", err);
    showToast("./assets/images/error_toast.png", "오류가 발생했습니다.");
    location.hash = "/"; // 오류 발생 시 메인 화면으로 복귀
>>>>>>> 95a4ef5ef8d1b6d8303c680f6c120e8fd1bb6601
  }
}