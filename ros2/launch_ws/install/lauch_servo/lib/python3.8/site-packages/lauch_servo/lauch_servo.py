import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import serial
from geometry_msgs.msg import Twist


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.subscription = self.create_subscription(
            Int64,
            'topic_py',
            self.listener_callback,
            10)

        self.teleop = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.teleop_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('degree: "%s"' % msg.degree)

    def teleop_callback(self, msg):
        self.get_logger().info(
            'linear: "%s","%s","%s" \n angular: "%d","%d","%d"' % 
            (msg.linear.x,msg.linear.y,msg.linear.z,
             msg.angular.x,msg.angular.y,msg.angular.z,))

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()