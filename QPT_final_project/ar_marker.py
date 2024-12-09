import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import threading
import math
from rclpy.qos import qos_profile_sensor_data
from collections import deque
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import socket

class AGVController(Node):
    def __init__(self):
        super().__init__('agv_controller_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # 속도 제한 설정
        self.limit_speed = 80  # 최대 속도 제한값
        self.min_pwm = 30  # 모터 속도의 최소값 (PWM)
        self.speed_scale = 1.5  # 속도 배율

        # 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)
            self.get_logger().info("Serial connection established on /dev/ttyAMA1 at 9600 baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial_port = None

        # /cmd_vel 토픽 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # /imu/data 토픽 퍼블리셔
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        
        # /cmd_vel 퍼블리셔 (카메라 제어 로직에서 /cmd_vel를 보낼 수 있게 함)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # 시리얼 수신 스레드 시작
        if self.serial_port:
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()

        # 자력계 보정 파라미터
        self.mag_bias = [-339.590589, 96.630322, -397.338474]
        self.mag_scale = [
            [0.122757, 0.005258, -0.009477],
            [0.005258, 0.120840, -0.005066],
            [-0.009477, -0.005066, 0.117316]
        ]

        # Yaw 이동 평균 필터
        self.N = 100
        self.yaw_values = deque(maxlen=self.N)

        # 카메라 및 마커 인식 관련 설정
        self.stop_threads = False
        self.searching_marker = True

        # 아루코 파라미터 설정
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.camera_matrix = np.array([[389.05937443, 0, 328.49977251],
                                       [0, 527.33816784, 238.71084069],
                                       [0, 0, 1]])
        self.dist_coeffs = np.array([0.14130158, -0.32542408, 0.00217276, -0.00148743, 0.19638934])
        self.marker_size = 0.055  # 55mm

        # UDP 설정
        self.udp_ip = '192.168.0.129'  # 수신 PC IP
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.max_packet_size = 60000

        # 카메라 스레드 시작
        self.camera_thread = threading.Thread(target=self.camera_capture)
        self.camera_thread.start()

    def cmd_vel_callback(self, msg):
        if not self.serial_port:
            self.get_logger().error("Serial port not available.")
            return

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        motor_command = []

        if linear_x == 0 and angular_z != 0:
            # 제자리 회전 모드
            rotation_speed = int(abs(angular_z) * self.limit_speed * self.speed_scale)
            rotation_speed = max(self.min_pwm, rotation_speed)
            rotation_speed = max(min(rotation_speed, self.limit_speed), -self.limit_speed)
            if angular_z < 0:
                motor_command = [0xf1, 'cc', rotation_speed]
            elif angular_z > 0:
                motor_command = [0xf1, 'cw', rotation_speed]
        else:
            base_speed = int(abs(linear_x) * self.limit_speed * self.speed_scale)
            base_speed = max(self.min_pwm, base_speed)
            base_speed = max(min(base_speed, self.limit_speed), -self.limit_speed)
            motor_command = [0xf0, base_speed, base_speed, base_speed, base_speed]

            reduction = int(abs(base_speed) * 0.55)
            boost = int(abs(base_speed) * 0.15)

            if angular_z < 0:
                motor_command[1] -= reduction
                motor_command[2] -= reduction
                motor_command[3] += boost
                motor_command[4] += boost
            elif angular_z > 0:
                motor_command[1] += boost
                motor_command[2] += boost
                motor_command[3] -= reduction
                motor_command[4] -= reduction

            if linear_x < 0:
                motor_command[1] = -motor_command[1]
                motor_command[2] = -motor_command[2]
                motor_command[3] = -motor_command[3]
                motor_command[4] = -motor_command[4]

            if linear_y != 0:
                lateral_speed = max(min(int(abs(linear_y) * self.limit_speed), self.limit_speed), -self.limit_speed)
                lateral_speed = int(lateral_speed * 0.5)
                if linear_y > 0:
                    motor_command[1] = -lateral_speed
                    motor_command[2] = lateral_speed
                    motor_command[3] = lateral_speed
                    motor_command[4] = -lateral_speed
                elif linear_y < 0:
                    motor_command[1] = lateral_speed
                    motor_command[2] = -lateral_speed
                    motor_command[3] = -lateral_speed
                    motor_command[4] = lateral_speed

        motor_command_str = ','.join(map(str, motor_command)) + '\n'
        self.serial_port.write(motor_command_str.encode())
        self.get_logger().info(
            f"Linear X: {linear_x}, Linear Y: {linear_y}, Angular Z: {angular_z}\n"
            f"Adjusted Motor Command List: {motor_command}\n"
            f"Commands sent to motors: {motor_command}"
        )

    def control_agv(self, distance, angle_deg):
        """각도/거리 상황에 따라 후진 곡선 주행을 포함한 로직 추가"""
        angle_difference = angle_deg - 90
        angle_threshold = 3.0   # 각도 허용 오차
        target_distance = 0.15
        distance_threshold = 0.03
        distance_error = distance - target_distance

        twist = Twist()

        # 추가 로직: 각도 오차가 크고(distance가 너무 짧은 상황)면 후진 곡선
        # 예: angle_difference가 10도 이상이고 distance < target_distance + 0.1 일 경우 뒤로 빼면서 각도 맞춤
        if abs(angle_difference) > 5 and distance < target_distance + 0.1:
            # 뒤로 물러나며 각도 조정
            linear_speed = -0.15  # 약간 후진
            # 각도 오차에 따라 회전 속도 설정 (좀 더 크게 회전)
            angle_speed = 0.5 * angle_difference / 90.0
            angle_speed = max(min(angle_speed, 0.5), -0.5)
            twist.linear.x = linear_speed
            twist.angular.z = angle_speed
        else:
            # 일반 곡선 주행 로직
            # 거리 오차 비례 전/후진 속도
            twist.linear.x = max(min(0.2 * distance_error, 0.2), -0.2)
            angle_speed = 0.4 * angle_difference / 90.0

            # 최소 회전 속도 확보
            if abs(angle_speed) < 0.05 and abs(angle_difference) > angle_threshold:
                angle_speed = 0.05 * (1 if angle_difference > 0 else -1)

            twist.angular.z = max(min(angle_speed, 0.5), -0.5)

            # 허용 범위 내 도달 시 정지
            if abs(distance_error) < distance_threshold and abs(angle_difference) < angle_threshold:
                self.get_logger().info("목표 위치에 도달: 정지")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)

    def read_serial_data(self):
        while self.serial_port and self.serial_port.is_open:
            try:
                line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                data = line.split(',')
                if data[0] == '243' and len(data) == 10:
                    try:
                        acc_x = float(data[1])
                        acc_y = float(data[2])
                        acc_z = float(data[3])
                        gyro_x = float(data[4])
                        gyro_y = float(data[5])
                        gyro_z = float(data[6])
                        mag_x = float(data[7])
                        mag_y = float(data[8])
                        mag_z = float(data[9])

                        raw_mag = [mag_x, mag_y, mag_z]
                        calib_mag = self.apply_mag_calibration(raw_mag)
                        yaw = self.calculate_yaw(calib_mag[0], calib_mag[1])

                        self.yaw_values.append(yaw)
                        filtered_yaw = sum(self.yaw_values) / len(self.yaw_values)

                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "imu_link"
                        imu_msg.linear_acceleration.x = acc_x * 9.80665
                        imu_msg.linear_acceleration.y = acc_y * 9.80665
                        imu_msg.linear_acceleration.z = acc_z * 9.80665
                        imu_msg.angular_velocity.x = math.radians(gyro_x)
                        imu_msg.angular_velocity.y = math.radians(gyro_y)
                        imu_msg.angular_velocity.z = math.radians(gyro_z)

                        self.imu_publisher.publish(imu_msg)

                        self.get_logger().debug(f"Yaw: {yaw:.2f} degrees, Filtered Yaw: {filtered_yaw:.2f} degrees")
                    except ValueError as ve:
                        self.get_logger().error(f"Value conversion error: {ve} with data: {data}")
                else:
                    self.get_logger().debug(f"Received unrecognized data: {line}")
            except Exception as e:
                self.get_logger().error(f"Error reading serial data: {e}")

    def apply_mag_calibration(self, raw_mag):
        temp = [raw_mag[i] - self.mag_bias[i] for i in range(3)]
        calib_mag = [0, 0, 0]
        for i in range(3):
            calib_mag[i] = sum(self.mag_scale[i][j] * temp[j] for j in range(3))
        return calib_mag

    def calculate_yaw(self, mag_x, mag_y):
        return math.atan2(mag_y, mag_x) * (180.0 / math.pi)

    def camera_capture(self):
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            self.stop_threads = True
            return

        while not self.stop_threads:
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error("프레임을 읽을 수 없습니다.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None and len(ids) > 0:
                aruco.drawDetectedMarkers(frame, corners, ids)
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], self.marker_size, self.camera_matrix, self.dist_coeffs)
                distance = np.linalg.norm(tvec[0][0])
                angle_to_marker = math.degrees(math.atan2(tvec[0][0][0], tvec[0][0][2]))
                angle_deg = angle_to_marker + 90  # 정면을 90도로 보정

                # 각도, 거리 화면 표시
                cv2.putText(frame, f"Angle: {angle_deg:.2f} deg", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
                cv2.putText(frame, f"Distance: {distance:.2f} m", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

                self.control_agv(distance, angle_deg)
                self.searching_marker = False
            else:
                self.search_for_marker()

            # UDP 전송
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            for i in range(0, len(buffer), self.max_packet_size):
                chunk = buffer[i:i + self.max_packet_size]
                self.sock.sendto(chunk, (self.udp_ip, self.udp_port))

        cap.release()
        cv2.destroyAllWindows()

    def search_for_marker(self):
        if self.searching_marker:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_vel_publisher.publish(twist)

    def destroy_node(self):
        self.stop_threads = True
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    agv_controller = AGVController()
    rclpy.spin(agv_controller)
    agv_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
