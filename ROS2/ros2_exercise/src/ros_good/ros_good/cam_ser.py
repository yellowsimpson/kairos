from cv_msg.srv import SrvGood
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SrvGood, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Server running...')
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        
        timer_period = 0.01  # Timer to update video frame
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
          
        if ret:
            cv2.imshow('win', frame)
            cv2.waitKey(1)
        self.get_logger().info('Publishing video frame')

    def add_two_ints_callback(self, request, response):
        ret, frame = self.cap.read()  # Capture current frame
        if ret:
            self.get_logger().info(f'Capturing frame at cmd: {request.cmd}')
            # Convert OpenCV image to ROS Image message
            response.image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        else:
            self.get_logger().warn('Failed to capture frame.')
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
