import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from modbus_frame import create_modbus_frame, setup_serial
import time
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class MoveItPlanningDemo:
    def __init__(self):
        rospy.init_node("moveit_demo")
        self.pose_x = None
        self.pose_y = None
        self.pose_z = None
        self.agv_x = 0.0
        self.agv_y = 0.0
        self.agv_theta = 0
        self.pub_init()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.x_dir = 0
        self.moter_dir = "03"
        self.previous_top_left_x = None
        self.ser_plc = setup_serial()

    def capture_center(self, motor_dir):
        # 색상 범위 설정 (HSV)
        color_ranges = {
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'red1': (np.array([0, 100, 100]), np.array([10, 255, 255])),
            'red2': (np.array([170, 100, 100]), np.array([180, 255, 255])),
            'black': (np.array([0, 0, 0]), np.array([180, 255, 30]))
        }

        selected_color = None  # 이 부분을 while 루프 밖으로 이동하여 한 번만 실행하도록 변경합니다.

        while True:
            top_left_x_yellow = None
            center_reds = []  # 여러 빨간색 중심을 저장할 리스트
            top_left_x_black = None

            ret, frame = self.cap.read()
            if not ret:
                print("Unable to read frame from camera.")
                return None

            camera_center_x = frame.shape[1] // 2
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            for color, (lower, upper) in color_ranges.items():
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                if color == 'red1' or color == 'red2':
                    if color == 'red1':
                        mask_red = mask
                    else:
                        mask_red = cv2.bitwise_or(mask_red, mask)
                    continue

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    rect = cv2.minAreaRect(largest_contour)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(frame, [box], 0, (255, 0, 255), 2)

                    if color == 'yellow':
                        top_left_x_yellow = min(box[0][0], box[1][0], box[2][0], box[3][0])
                    elif color == 'black':
                        top_left_x_black = min(box[0][0], box[1][0], box[2][0], box[3][0])

            if 'mask_red' in locals():
                contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    if cv2.contourArea(contour) > 500:  # 임계값 설정 (필요에 따라 조정)
                        rect = cv2.minAreaRect(contour)
                        center_red = rect[0]
                        center_red = (int(center_red[0]), int(center_red[1]))
                        center_reds.append(center_red)
                        cv2.circle(frame, center_red, 5, (0, 255, 0), -1)

            if selected_color is None:  # selected_color가 None일 때만 선택합니다.
                if motor_dir == "05":
                    if top_left_x_yellow is not None and top_left_x_black is not None:
                        # 오른쪽에 있는 색을 골라야함
                        if top_left_x_yellow < top_left_x_black:
                            selected_color = "black"
                        elif top_left_x_yellow > top_left_x_black:
                            selected_color = "yellow"
                elif motor_dir == "06":
                    if top_left_x_yellow is not None and top_left_x_black is not None:
                        # 왼쪽에 있는 색 골라야함
                        if top_left_x_yellow < top_left_x_black:
                            selected_color = "yellow"
                        elif top_left_x_yellow > top_left_x_black:
                            selected_color = "black"

            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

            # 여러 개의 빨간색 중심이 있을 때 가장 왼쪽 또는 오른쪽에 있는 것을 찾기
            if center_reds:
                if motor_dir == "05":
                    center_reds.sort(key=lambda center: center[0])  # x좌표를 기준으로 오름차순 정렬
                elif motor_dir == "06":
                    center_reds.sort(key=lambda center: center[0], reverse=True)  # x좌표를 기준으로 내림차순 정렬

                closest_center_red = center_reds[0]
                if abs(closest_center_red[0] - camera_center_x) < 10:
                    print("Red center is aligned with camera center.")
                    break

            current_top_left_x = None
            if selected_color == "yellow":
                current_top_left_x = top_left_x_yellow
            elif selected_color == "black":
                current_top_left_x = top_left_x_black

            if current_top_left_x is not None and self.previous_top_left_x is not None:
                change = current_top_left_x - self.previous_top_left_x
                width = cv2.norm(box[0] - box[1]) if selected_color is not None else None

                if width is not None and width != 0:
                    self.x_dir += change * 0.025 / width
                    print(self.x_dir)
                else:
                    print("Width is None or zero, cannot calculate.")

            self.previous_top_left_x = current_top_left_x
            time.sleep(0.5)
            try:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                break

    def start(self):
        self.moving(-0.134, -0.185, 0.338)
        rospy.sleep(1)

    def start_2(self):
        self.moving(-0.134, -0.2, 0.398)
        rospy.sleep(1)

    def plc_motor_ctrl(self, vec):
        frame = create_modbus_frame([0x3A, '01', vec, "00010008", 0x0D, 0x0A], 'lrc')
        self.ser_plc.write(frame)
        self.capture_center(vec)

        frame = create_modbus_frame([0x3A, '01', "03", "00010008", 0x0D, 0x0A], 'lrc')
        self.ser_plc.write(frame)

        time.sleep(2)

    def pub_init(self):
        self.pub_agv = rospy.Publisher('agv_position_orientation', Float32MultiArray, queue_size=1)

    def pub_coordinates(self):
        agv_msg = Float32MultiArray()
        agv_msg.data = [self.agv_x, self.agv_y, self.agv_theta]
        self.pub_agv.publish(agv_msg)
        rospy.loginfo(f"AGV position and orientation: x={self.agv_x}, y={self.agv_y}, theta={self.agv_theta}")

if __name__ == "__main__":
    o = MoveItPlanningDemo()
    o.plc_motor_ctrl("06")
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
