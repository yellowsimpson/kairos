from cv_msg.srv import SrvArduino
import rclpy
from rclpy.node import Node
import serial

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.srv = self.create_service(SrvArduino, 'motor_con', self.motor_callback)

    def motor_callback(self, request, response):
        #response.answer = request.dir + request.speed
        if request.dir == 1:
            # arduino LED on 
            self.ser.write('a'.encode())
        elif request.dir == 2:
            # arduino LED off 
            self.ser.write('b'.encode())
        response.answer = 45
        self.get_logger().info('Incoming request\ndirection: %d speed: %d' % (request.dir, request.speed))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()