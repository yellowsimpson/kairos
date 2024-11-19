import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist

class TeleopToArduino(Node):

    def __init__(self):
        super().__init__('teleop_to_arduino')

        # cmd_vel 메시지 구독
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

        # 아두이노 시리얼 포트 설정
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # 포트는 상황에 따라 수정
            self.get_logger().info("아두이노와 성공적으로 연결되었습니다.")
        except serial.SerialException as e:
            self.get_logger().error(f"아두이노 연결 실패: {e}")

    def listener_callback(self, msg):
        # linear.x 값에 따라 LED 켜기/끄기
        if msg.linear.x >1:
            self.get_logger().info('LED 켜기: 아두이노에 1 전송')
            self.arduino.write(b'1')  # '1'을 아두이노로 전송
        else:
            self.get_logger().info('LED 끄기: 아두이노에 0 전송')
            self.arduino.write(b'0')  # '0'을 아두이노로 전송

def main(args=None):
    rclpy.init(args=args)
    teleop_to_arduino = TeleopToArduino()

    rclpy.spin(teleop_to_arduino)

    # 종료 시 시리얼 포트 닫기
    teleop_to_arduino.arduino.close()
    teleop_to_arduino.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
