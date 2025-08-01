# roomie_ac/test/test_vs_client_node.py

import rclpy
import asyncio
from src.roomie_ac.roomie_ac.vision_client import VisionServiceClient # roomie_ac 패키지 내 모듈 임포트
from roomie_ac import config # config 임포트

async def test_vision_service_client():
    """
    VisionServiceClient를 초기화하고 더미 Vision Service에 요청을 보냅니다.
    """
    print("VisionServiceClient 테스트 노드 시작.")
    client_node = VisionServiceClient()

    # config.py에서 로봇 ID 가져오기
    robot_id = config.ROBOT_ID
    # 테스트할 버튼 ID
    # 102: 열기 버튼, 1: 1층 버튼 (ButtonStatus.srv 명세 참조)
    test_button_ids = [102, 1, 5] 

    print(f"\n--- VS 서비스 요청 테스트 시작 ---")
    response = await client_node.request_button_status(robot_id, test_button_ids)

    if response:
        print("\n--- VS 서비스 응답 성공 ---")
        print(f"Robot ID: {response.robot_id}")
        print(f"Success: {response.success}")
        for i, btn_id in enumerate(test_button_ids):
            # 응답 배열 인덱스 확인
            if i < len(response.xs):
                print(f"  버튼 {btn_id}: X={response.xs[i]:.2f}, Y={response.ys[i]:.2f}, Depth={response.depths[i]:.2f}, Pressed={response.is_pressed[i]}, Timestamp={response.timestamp[i].sec}.{response.timestamp[i].nanosec}")
            else:
                print(f"  버튼 {btn_id}에 대한 응답 데이터가 부족합니다.")
    else:
        print("\n--- VS 서비스 응답 실패 또는 오류 발생 ---")

    client_node.destroy_node()
    print("VisionServiceClient 테스트 노드 종료.")

def main(args=None):
    rclpy.init(args=args)
    # asyncio.run을 사용하여 비동기 함수 실행
    asyncio.run(test_vision_service_client())
    # rclpy.shutdown()은 asyncio.run 내부에서 처리되므로 여기서는 생략 가능

if __name__ == '__main__':
    main()