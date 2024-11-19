from cv_msg.srv import SrvArduino
import rclpy
from rclpy.node import Node
import serial

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)  # 시리얼 포트를 설정하세요
        self.srv = self.create_service(SrvArduino, 'motor_con', self.motor_callback)

    def motor_callback(self, request, response):
        if request.dir == 'f':
            command = f'f {request.speed}'  # 정방향 ('f ' + 속도)
        elif request.dir == 'r':
            command = f'r {request.speed}'  # 역방향 ('r ' + 속도)

        self.ser.write(command.encode())  # 아두이노로 명령 전송
        response.answer = request.speed  # 응답으로 속도 반환
        self.get_logger().info(f'Received command: {command}')

        return response

def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
