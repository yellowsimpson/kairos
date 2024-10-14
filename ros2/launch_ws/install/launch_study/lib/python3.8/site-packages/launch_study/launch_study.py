import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Subscriber 클래스 정의
class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # 'teleop_twist_keyboard'의 기본 토픽 이름
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received speed: {msg.linear}, turn: {msg.angular}')
        
    def timer_callback(self):
        msg = String()
        
def main(args=None):
    rclpy.init(args=args)

    # 노드 실행
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # 노드 종료
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()