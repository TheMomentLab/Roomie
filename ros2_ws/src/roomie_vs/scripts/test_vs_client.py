import rclpy
from rclpy.node import Node
from roomie_msgs.srv import SetVSMode
import sys

class VisionServiceClient(Node):
    """VisionNode의 SetVSMode 서비스를 테스트하기 위한 간단한 클라이언트입니다."""

    def __init__(self):
        super().__init__('vision_service_client')
        # SetVSMode 서비스 클라이언트 생성
        self.client = self.create_client(SetVSMode, '/vs/command/set_vs_mode')
        
        # 서비스 서버가 준비될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def send_request(self, mode_id):
        """서비스 요청을 보내고 응답을 기다립니다."""
        request = SetVSMode.Request()
        request.mode_id = mode_id
        
        # 비동기적으로 요청을 보내고 future 객체를 반환받음
        self.future = self.client.call_async(request)
        
        # future가 완료될 때까지 대기
        rclpy.spin_until_future_complete(self, self.future)
        
        try:
            response = self.future.result()
            if response.success:
                self.get_logger().info(f'모드 {mode_id}로 성공적으로 변경되었습니다.')
            else:
                self.get_logger().error(f'모드 {mode_id}로 변경하는 데 실패했습니다.')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {e}')

def main(args=None):
    """메인 함수: 노드를 초기화하고 서비스 요청을 보냅니다."""
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("사용법: ros2 run roomie_vs test_vs_client <mode_id>")
        return

    try:
        mode_id = int(sys.argv[1])
    except ValueError:
        print("오류: mode_id는 반드시 정수여야 합니다.")
        return

    client_node = VisionServiceClient()
    client_node.send_request(mode_id)

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()