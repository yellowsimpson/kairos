import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class joint_state_publisher_test(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(3.0, self.publish_joint_state)  # 3초 주기로 발행
        self.joint_angle = 0
        self.get_logger().info("start")

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3','joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6',
    'gripper_controller','gripper_base_to_gripper_left2','gripper_left3_to_gripper_left1','gripper_base_to_gripper_right3','gripper_base_to_gripper_right2',
    'gripper_right3_to_gripper_right1']

        if self.joint_angle == 0:
            joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_angle = 1
        else:
            joint_state.position = [1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_angle = 0

        self.publisher_.publish(joint_state)
        self.get_logger().info(f"joint state: {joint_state.position}")

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = joint_state_publisher_test()
    rclpy.spin(joint_state_publisher)

if __name__ == '__main__':
    main()

