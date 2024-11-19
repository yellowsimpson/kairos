from uga_msg.srv import SrvServo
import rclpy
from rclpy.node import Node
import serial

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # 시리얼 포트를 설정하세요
        self.srv = self.create_service(SrvServo, 'servo_con', self.servo_callback)

    def servo_callback(self, request, response):
        response.result =request.degree
        
        self.get_logger().info('Incoming requset \n degree: %d'%request.degree)
        angle=str(request.degree)+'\n'
        self.ser.write(angle.encode())
        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
