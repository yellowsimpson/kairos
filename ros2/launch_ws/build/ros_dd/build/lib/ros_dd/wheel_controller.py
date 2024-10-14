# ros_dd/wheel_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        # 파라미터 설정
        self.wheel_radius = 0.15  # 바퀴 반지름 (미터 단위)
        self.wheel_separation = 0.6  # 왼쪽과 오른쪽 바퀴 사이의 거리

        # 구독자 설정
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # 퍼블리셔 설정
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 타이머를 이용하여 조인트 상태 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # 내부 상태 변수
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        self.current_joint_state = JointState()
        self.current_joint_state.name = ['wheel1_joint', 'wheel2_joint', 'wheel3_joint', 'wheel4_joint']
        self.current_joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.current_joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        # Twist 메시지로부터 바퀴 속도 계산
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # 차동 주행 모델을 이용한 속도 계산
        self.left_wheel_vel = (linear_vel - (self.wheel_separation / 2.0) * angular_vel) / self.wheel_radius
        self.right_wheel_vel = (linear_vel + (self.wheel_separation / 2.0) * angular_vel) / self.wheel_radius

    def publish_joint_states(self):
        # 바퀴 위치 업데이트
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 나노초를 초로 변환
        self.last_time = current_time

        # 속도를 기반으로 위치 변화 계산
        delta_left = self.left_wheel_vel * dt
        delta_right = self.right_wheel_vel * dt

        # 위치 업데이트 (wheel1과 wheel3은 왼쪽 바퀴, wheel2와 wheel4는 오른쪽 바퀴로 가정)
        self.current_joint_state.position[0] += delta_left  # wheel1_joint
        self.current_joint_state.position[2] += delta_left  # wheel3_joint
        self.current_joint_state.position[1] += delta_right  # wheel2_joint
        self.current_joint_state.position[3] += delta_right  # wheel4_joint

        # 속도 업데이트
        self.current_joint_state.velocity[0] = self.left_wheel_vel
        self.current_joint_state.velocity[2] = self.left_wheel_vel
        self.current_joint_state.velocity[1] = self.right_wheel_vel
        self.current_joint_state.velocity[3] = self.right_wheel_vel

        # 타임스탬프 업데이트
        self.current_joint_state.header.stamp = self.get_clock().now().to_msg()

        # 조인트 상태 퍼블리시
        self.joint_state_pub.publish(self.current_joint_state)

def main(args=None):
    rclpy.init(args=args)
    wheel_controller = WheelController()
    rclpy.spin(wheel_controller)
    wheel_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
