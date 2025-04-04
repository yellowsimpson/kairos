#!/usr/bin/env python3
import rospy, roslib, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler
from modbus_frame import create_modbus_frame, setup_serial
import time
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from moveit_commander.conversions import list_to_pose

class MoveItPlanningDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
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
        self.x_position = 0
        self.motor_dir = "03"
        self.previous_top_left_x = None
        self.ser_plc = setup_serial()
        self.offset = 0.02

        self.load_robot_description()
        self.x_distance_center_red=0
        self.y_distance_center_red=0

        # Initialize the MoveGroupCommander for arm and gripper
        self.arm = moveit_commander.MoveGroupCommander("arm_group", ns="/cobot")
        self.gripper = moveit_commander.MoveGroupCommander("gripper_group", ns="/cobot")

        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = "base"
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.home = self.arm.get_current_pose(end_effector_link='link6').pose

        self.pub_init()

    def load_robot_description(self):
        # Load URDF
        urdf_path = rospy.get_param('~urdf_path', '/home/jsy/jsy_ws/src/mycobot_ros/mycobot_description/urdf/mycobot_320_gripper.urdf')
        with open(urdf_path, 'r') as urdf_file:
            urdf_string = urdf_file.read()
        rospy.set_param('/robot_description', urdf_string)

        # Load SRDF
        srdf_path = rospy.get_param('~srdf_path', '/home/jsy/jsy_ws/src/mycobot_ros/mycobot_320/mycobot_320_gripper_moveit/config/firefighter.srdf')
        with open(srdf_path, 'r') as srdf_file:
            srdf_string = srdf_file.read()
        rospy.set_param('/robot_description_semantic', srdf_string)

    def jmove_to_pose_goal(self, pose_goal): #pose goal
        self.arm.set_pose_target(pose_goal)
        success, plan, _, _ = self.arm.plan()
        if success:
            self.arm.execute(plan, wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def jmove_to_joint_goal(self, joint_goal): #joint goal
        self.arm.set_joint_value_target(joint_goal)
        success, plan, _, _ = self.arm.plan()
        if success:
            self.arm.execute(plan, wait=True)
        self.arm.stop()

    def close(self): #그리퍼닫기
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)
        self.gripper.stop()

    def open(self): #그리퍼 열기
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        self.gripper.stop()
        
    def capture_center(self, motor_dir):
        rospy.sleep(1)
        i=1
        # 색상 범위 설정 (HSV)
        color_ranges = {
            # 'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255])),
            'yellow': (np.array([130, 50, 50]), np.array([170, 255, 255])), # 이거 보라색임
            'red1': (np.array([0, 100, 100]), np.array([10, 255, 255])),
            'red2': (np.array([170, 100, 100]), np.array([180, 255, 255])),
            # 'black': (np.array([0, 0, 0]), np.array([180, 255, 30])),
            'black': (np.array([90, 50, 100]), np.array([130, 255, 255])), #파란색임
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
                    i=2
                    if top_left_x_yellow is not None and top_left_x_black is not None:
                        # 오른쪽에 있는 색을 골라야함
                        if top_left_x_yellow < top_left_x_black:
                            selected_color = "black"
                        elif top_left_x_yellow > top_left_x_black:
                            selected_color = "yellow"
                elif motor_dir == "06":
                    i=1
                    if top_left_x_yellow is not None and top_left_x_black is not None:
                        # 왼쪽에 있는 색 골라야함
                        if top_left_x_yellow < top_left_x_black:
                            selected_color = "yellow"
                        elif top_left_x_yellow > top_left_x_black:
                            selected_color = "black"

            # cv2.imshow("Frame", frame)
            cv2.waitKey(1)
            
            # 여러 개의 빨간색 중심이 있을 때 가장 왼쪽 또는 오른쪽에 있는 것을 찾기
            if center_reds:
                if motor_dir == "05":
                    center_reds.sort(key=lambda center: center[0])  # x좌표를 기준으로 오름차순 정렬
                elif motor_dir == "06":
                    center_reds.sort(key=lambda center: center[0], reverse=True)  # x좌표를 기준으로 내림차순 정렬

                for center_red in center_reds:
                    if abs(center_red[0] - camera_center_x) < 10:
                        print("Red center is aligned with camera center.")
                        return

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
                    self.x_dir += change * 0.025*i / width
                    print(self.x_dir)
                else:
                    print("Width is None or zero, cannot calculate.")

            self.previous_top_left_x = current_top_left_x
            time.sleep(0.2)
            try:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except KeyboardInterrupt:
                break

    def moving(self, pose_x, pose_y, pose_z, rx , ry, rz):
        # Set the target position for Cartesian motion
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = pose_x
        target_pose.pose.position.y = pose_y
        target_pose.pose.position.z = pose_z
        q = quaternion_from_euler(rx, ry, rz)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        # Set the target position for Cartesian motion
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        print(self.arm.get_current_pose(end_effector_link='link6').pose)
        plan = self.arm.plan()
        self.arm.execute(plan[1])  # Modified line
        rospy.sleep(1)

    def plc_motor_ctrl(self, vec):
        frame = create_modbus_frame([0x3A, '01', vec, "00010008", 0x0D, 0x0A], 'lrc')
        self.ser_plc.write(frame)
        time.sleep(0.4)
        self.ser_plc.write(frame)

        self.capture_center(vec)

        frame = create_modbus_frame([0x3A, '01', "03", "00010008", 0x0D, 0x0A], 'lrc')
        self.ser_plc.write(frame)
        time.sleep(0.4)
        self.ser_plc.write(frame)
        self.x_position = 0

    def pub_init(self):
        self.pub_agv = rospy.Publisher('agv_position_orientation', Float32MultiArray, queue_size=1)

    def pub_coordinates(self):
        agv_msg = Float32MultiArray()
        agv_msg.data = [self.agv_x, self.agv_y, self.agv_theta]
        self.pub_agv.publish(agv_msg)
        rospy.loginfo(f"AGV position and orientation: x={self.agv_x}, y={self.agv_y}, theta={self.agv_theta}")

    def get_red_center_offset_once(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Unable to read frame from camera.")
            return None, None, None  # x_offset, y_offset, width

        camera_center_x = frame.shape[1] // 2
        camera_center_y = frame.shape[0] // 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 색상 범위 설정 (HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 빨간색 범위 마스크 적용
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        # 빨간색 마스크로 윤곽선 찾기
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_reds = []

        for contour in contours:
            if cv2.contourArea(contour) > 500:  # 면적이 일정 크기 이상인 경우에만 처리
                rect = cv2.minAreaRect(contour)
                center_red = rect[0]
                center_red = (int(center_red[0]), int(center_red[1]))
                center_reds.append(center_red)
                cv2.circle(frame, center_red, 5, (0, 255, 0), -1)

                # 빨간색 물체의 외곽을 감싸는 최소 크기의 사각형 그리기
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                # 가로 한 변의 길이 측정
                width = np.sqrt(np.sum((box[0] - box[1]) ** 2))

        if center_reds:
            closest_center_red = min(center_reds, key=lambda center: abs(center[0] - camera_center_x) + abs(center[1] - camera_center_y))
            x_offset = (-(closest_center_red[0] - camera_center_x) * 0.05) / width
            y_offset = (-(closest_center_red[1] - camera_center_y) * 0.05)  / width + 0.04

            # 웹캠 이미지에 빨간색 물체 및 중심 표시
            cv2.circle(frame, (camera_center_x, camera_center_y), 5, (255, 0, 0), -1)
            cv2.putText(frame, f"Camera Center", (camera_center_x - 50, camera_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            cv2.circle(frame, closest_center_red, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"Red Center", (closest_center_red[0] - 50, closest_center_red[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # 웹캠 이미지 보여주기
            # cv2.imshow("Frame", frame)
            # cv2.waitKey(0)

            return x_offset, y_offset
        else:
            print("No red center found.")
            return 0, 0


    def tmove_to_pose_goal(self, pose_st, pose_goal): #직선이동
        waypoints = []
        waypoints.append(pose_st)
        waypoints.append(pose_goal)
        plan, fraction = self.arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        if fraction == 1.0:
            self.arm.execute(plan, wait=True)


    def arm_get_current_pose(self):
        print(self.arm.get_current_pose().pose)



if __name__ == "__main__":
    o = MoveItPlanningDemo()
    
    # for i in range(0,3):
    #     rospy.sleep(1)
    #     o.open()
    #     cam_joint_init= [-90, 65, -68, 3, 90, 0]
    #     cam_joint_init_rad = np.deg2rad(cam_joint_init)
    #     o.jmove_to_joint_goal(cam_joint_init_rad)
    #     rospy.sleep(3)
    #     print("arm init pose")

    #     #plc 모터 작동
    #     o.plc_motor_ctrl("05")

    #     rospy.sleep(3)
    #     a,b = o.get_red_center_offset_once()


    #     print('잡기 위치')
    #     o.moving(-0.08+a, -0.145, 0.45, 0.0, 0.0, 0.0)
    #     rospy.sleep(3)

    #     print('그리퍼 잡기')
    #     o.moving(-0.08+a, -0.145, 0.45+b, 0.0, 0.0, 0.0)
    #     rospy.sleep(3)
    #     o.close()

    #     cam_joint_init= [-90, 65, -68, 3, 90, 0]
    #     cam_joint_init_rad = np.deg2rad(cam_joint_init)
    #     o.jmove_to_joint_goal(cam_joint_init_rad)
    #     rospy.sleep(3)

    #     print('토마토 놓을위치')
    #     gripper_open_posetion= [120, 10, -60, -40, 90, -65]
    #     gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    #     o.jmove_to_joint_goal(gripper_open_posetion_rad)
    #     rospy.sleep(2)

    #     print('토마토 놓기')
    #     o.open()
    
    # #역가 
    # rospy.loginfo("agv 0")
    # o.pub_coordinates()
    # rospy.sleep(1)

    # rospy.loginfo("agv 1")
    # o.agv_x = 0.5
    # o.agv_y = 0
    # o.agv_theta = 1.5709
    # o.pub_coordinates()
    # rospy.sleep(1)
    
    # rospy.loginfo("agv 2")
    # o.agv_x = 0.1
    # o.agv_y = -1.3
    # o.agv_theta = 0
    # o.pub_coordinates()
    # rospy.sleep(1)

    # rospy.loginfo("agv 3")
    # o.agv_x = 0.81
    # o.agv_y = -1.25
    # o.agv_theta = 1.5709
    # o.pub_coordinates()
    # rospy.sleep(1)

    #---------------

    print('토마토 바스켓 잡을 위치')
    gripper_open_posetion= [131, 11, -73, -27, 90, 43]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)

    o.arm_get_current_pose()

    print('토마토 바스켓 잡을 위치')
    gripper_open_posetion= [131, 16, -94, -10, 90, 43]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)

    o.close()

    print('박스 들기')
    gripper_open_posetion= [128, -10, -21, -60, 92, 40]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)

    time.sleep(3)

    print('박스 엎기 위치')
    gripper_open_posetion= [121, -39, -50, 65, 59, 12]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)


    print('박스 놓기 위치')
    gripper_open_posetion= [128, -10, -21, -60, 92, 40]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)
    

    print('토마토 바스켓 잡을 위치')
    gripper_open_posetion= [131, 11, -73, -27, 90, 43]
    gripper_open_posetion_rad = np.deg2rad(gripper_open_posetion)
    o.jmove_to_joint_goal(gripper_open_posetion_rad)
    rospy.sleep(2)

    o.open()


    # rospy.loginfo("agv 4")
    # o.agv_x = 0.1
    # o.agv_y = -1.3
    # o.agv_theta = 1.5709
    # o.pub_coordinates()
    # rospy.sleep(1)


    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)
