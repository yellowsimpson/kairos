import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_py', 10)
        self.teleop = self.create_subscription{
            Twist,
            'cmd_vel', #Here we want to hear teleop: To hear teleop we shtto use cmd vel
            self.teleop_callback

        }
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def teleop_callback(self, msg):
        angle = msg.angular.z
        self.get_logger().info('Publishing: "%s"' % angele)
        if angele

        self.i += 1
    
    def teleop_callback(self, ):

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
