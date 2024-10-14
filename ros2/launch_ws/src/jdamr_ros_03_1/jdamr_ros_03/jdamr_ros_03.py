import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')  # 노드 이름 지정
        self.subscription = self.create_subscription(
            JointState,             # 메시지 타입
            'cmd_vel',              # 구독할 토픽 이름
            self.listener_callback, # 콜백 함수, 받은 데이터 그냥 찔러주는거
            10                      # 큐 크기
        )
        self.subscription  # prevent unused variable warning
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
            )
        self.joint_timer = self.create_timer(
            0.5,
            self.joint_timer_cb)
        self.joint_pub = 0

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')

    def joint_timer_cb(self):
        joint += 0.1
        if joint > 3.14:
            joint = 3.14
        self.get_logger().info(joint)
        join = JointState()
        join.position[0] = self.joint
        self.joint_pub.publish(join)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    try:
        rclpy.spin(minimal_subscriber)  # 노드가 종료될 때까지 계속 실행
    except KeyboardInterrupt:
        pass

    # 노드 종료 시 정리
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
