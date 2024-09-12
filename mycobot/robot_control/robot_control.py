import threading
import time
import cv2
import numpy as np
from tensorflow.keras.models import load_model
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input  # 전처리 추가
from pymycobot.mycobot import MyCobot
# MyCobot 초기화
mc = MyCobot('COM5', 115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
mc.set_gripper_calibration()
# 카메라 초기화
cap = cv2.VideoCapture(0)
class_labels = ["purple", "red", "yellow"]
model = load_model("uga.h5")
# 전역 변수로 물체의 위치를 추적
target_position = None
robot_ready_for_detection = False
color_detected = None  # 인식된 색상 추적
def move_to_initial_position():
    # 초기 위치로 이동
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(3)
def move_to_detection_position():
    # 물체를 인식할 위치로 이동
    mc.send_angles([55.11, 66.81, 12.81, 4.87, -92.1, -17.22], 20)
    time.sleep(3)
def move_to_drop_position():
    # 물체를 놓기 위한 위치로 이동
    mc.send_angles([-8.26, 63.54, -24.69, 32.34, -83.49, -14.67], 20)
    time.sleep(3)
def grab_object():
    # 그리퍼 닫기 (물체 잡기)
    mc.set_eletric_gripper(1)
    mc.set_gripper_value(5, 20, 1)
    time.sleep(1)
def release_object():
    # 그리퍼 열기 (물체 놓기)
    mc.set_gripper_value(100, 20, 1)
    time.sleep(1)
def control_robot():
    global target_position, robot_ready_for_detection, color_detected
    while True:
        # 1. 초기 위치로 이동
        move_to_initial_position()
        # 2. 물체를 인식할 위치로 이동
        move_to_detection_position()
        # 물체 인식 준비 완료
        robot_ready_for_detection = True
        while target_position is None or color_detected != 'red':
            time.sleep(0.1)  # 물체 인식 대기
        # 3. 물체 잡기
        grab_object()
        color_detected = None
        print(color_detected)
        # 4. 물체를 놓을 위치로 이동
        move_to_drop_position()
        print("drop")
        # 5. 물체 놓기
        release_object()
        print("release")
        # 6. 다음 물체 인식을 위해 초기화
        robot_ready_for_detection = False
        target_position = None
def camera_recognition():
    global target_position, robot_ready_for_detection, color_detected
    while True:
        if robot_ready_for_detection:  # 로봇이 인식 위치에 도착한 후에만 카메라 인식 시작
            ret, img = cap.read()
            if ret:
                # 이미지를 224x224로 크기 조정
                img_resized = cv2.resize(img, (224, 224))
                # MobileNetV2 전처리 적용
                img_preprocessed = preprocess_input(img_resized)
                # 모델 입력 차원 추가
                X = np.expand_dims(img_preprocessed, axis=0)
                # 예측 실행
                s = model(X, training=False)
                index = np.argmax(s)
                # 기본값 설정
                strr = "unknown"
                if index == 0:
                    strr = 'purple'
                    color_detected = 'purple'
                elif index == 1:
                    strr = 'red'
                    color_detected = 'red'
                    target_position = (100, 100)  # 실제 환경에 맞게 조정
                elif index == 2:
                    strr = 'yellow'
                    color_detected = 'yellow'
                cv2.putText(img, strr, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                cv2.imshow('win', img)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    break
# 로봇 제어 스레드 생성 및 실행
robot_thread = threading.Thread(target=control_robot)
robot_thread.start()
# 카메라 인식 스레드 생성 및 실행
camera_thread = threading.Thread(target=camera_recognition)
camera_thread.start()
# 스레드 종료 대기
camera_thread.join()
# 카메라 릴리스 및 창 닫기
cap.release()
cv2.destroyAllWindows()
# 로봇 제어 스레드 종료
robot_thread.join()
