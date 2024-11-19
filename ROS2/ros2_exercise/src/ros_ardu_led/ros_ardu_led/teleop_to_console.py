import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopToConsole(Node):

    def __init__(self):
        super().__init__('teleop_to_console')
        
        # cmd_vel 메시지 구독
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
    
    # 메시지를 수신하여 linear.x 값을 출력하는 콜백 함수
    def listener_callback(self, msg):
        self.get_logger().info(f"Received linear.x: {msg.linear.x}")

def main(args=None):
    rclpy.init(args=args)
    teleop_to_console = TeleopToConsole()

    rclpy.spin(teleop_to_console)

    # 노드 종료 시 자원 해제
    teleop_to_console.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
