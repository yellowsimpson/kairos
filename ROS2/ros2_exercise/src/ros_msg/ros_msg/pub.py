import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_msg.msg import MsgCenter
import random

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MsgCenter, 'topic_py_jh', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MsgCenter()
        msg.x=random.randint(0,50)
        msg.y=random.randint(0,50)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d %d"' % (msg.x,msg.y))
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()