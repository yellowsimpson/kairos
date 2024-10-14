from cv_msg.srv import SrvGood

import rclpy
from rclpy.node import Node
import cv2

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SrvGood, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Server running...')
        self.cap = cv2.VideoCapture(0)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def timer_callback(self):
        ret, frame = self.cap.read()    
        if ret == True:
            cv2.imshow('win', frame)
            cv2.waitKey(1)
        self.get_logger().info('video frame')


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()