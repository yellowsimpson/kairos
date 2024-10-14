import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class WheelPublisher(Node):
    def __init__(self):
        super().__init__('wheel_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # 0.1초마다 콜백 함수 호출
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angle = 0.0  # 초기 각도
        self.get_logger().info('Wheel Publisher Node has been started.')

    def timer_callback(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()

        # URDF에서 정의한 바퀴 관절 이름 사용
        joint_state.name = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint']

        # 각 바퀴의 위치(각도) 설정
        # 여기서는 모든 바퀴가 동일한 속도로 회전한다고 가정
        joint_state.position = [self.angle, self.angle, self.angle, self.angle]

        # 관절 상태 퍼블리시
        self.publisher_.publish(joint_state)

        # 각도 업데이트 (예: 0.1 라디안씩 증가)
        self.angle += 0.1

        # 각도 범위 제한 (0 ~ 2*pi)
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

def main(args=None):
    rclpy.init(args=args)
    node = WheelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
