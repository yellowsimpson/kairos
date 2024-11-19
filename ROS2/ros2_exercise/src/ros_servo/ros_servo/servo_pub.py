import rclpy
from rclpy.node import Node
from uga_msg.msg import MsgServo

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MsgServo, 'topic_servo_cjh', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MsgServo()
        msg.degree = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.degree)
        self.i += 5
        if self.i > 90:
            self.i = 0

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
