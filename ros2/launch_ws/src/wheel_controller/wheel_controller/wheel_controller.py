import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_state)  # 0.1초마다 업데이트
        self.angle = 0.0  # 초기 각도

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_joint']  # 바퀴의 조인트 이름
        self.angle += 0.1  # 각도 증가 (회전 속도 조절)
        msg.position = [math.sin(self.angle)]  # 사인 함수를 이용해 회전
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    wheel_controller = WheelController()
    rclpy.spin(wheel_controller)
    wheel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
