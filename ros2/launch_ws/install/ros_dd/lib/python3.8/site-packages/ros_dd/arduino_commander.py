# ros_dd/arduino_commander.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ArduinoCommander(Node):
    def __init__(self):
        super().__init__('arduino_commander')
        
        # 시리얼 포트 설정 (실제 포트 이름으로 변경하세요)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        
        # `/cmd_vel` 토픽 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
    def cmd_vel_callback(self, msg):
        # linear.x 값에 따라 명령 결정
        if msg.linear.x > 0:
            command = 'F'  # 전진
        elif msg.linear.x < 0:
            command = 'B'  # 후진
        else:
            command = 'S'  # 정지

        # Arduino로 명령 전송
        try:
            self.ser.write((command + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent command: {command}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

            

def main(args=None):
    rclpy.init(args=args)
    arduino_commander = ArduinoCommander()
    rclpy.spin(arduino_commander)
    arduino_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
