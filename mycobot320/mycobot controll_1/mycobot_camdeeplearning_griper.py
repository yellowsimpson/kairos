import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array
import time

# MyCobot 설정
mc = MyCobot('COM5', 115200)
mc.init_gripper()
mc.set_gripper_calibration()

# 학습된 모델 로드 (예: 'color_detection_model.h5')
model = load_model('colorblock.h5')

def is_red(prediction):
    """예측 결과에서 빨간색이 포함되어 있는지 확인"""
    return prediction[0] > max(prediction[1], prediction[2])

def preprocess_image(img):
    """이미지를 모델에 맞게 전처리"""
    img = cv2.resize(img, (64, 64))  # 학습 시 사용한 이미지 크기에 맞추기
    img = img_to_array(img)
    img = np.expand_dims(img, axis=0)
    img = img / 255.0  # 정규화 (모델 학습 시 적용된 정규화와 일치하게)
    return img

def capture_image():
    """카메라로부터 이미지 캡처"""
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        cv2.imwrite('frame.jpg', frame)
    cap.release()
    return frame

def move_and_grip():
    """로봇을 이동시키고 그리퍼를 제어"""
    # 첫 번째 각도로 이동
    print("첫 번째 각도로 이동 중...")
    mc.send_angle(1, 90, 20)
    time.sleep(3)
    mc.send_angle(5, -90, 20)
    time.sleep(3)
    mc.send_angle(2, 80, 20)
    time.sleep(3)
    
    # 카메라로부터 이미지 캡처
    print("카메라를 켜서 이미지 캡처 중...")
    frame = capture_image()
    preprocessed_img = preprocess_image(frame)
    
    # 예측 수행
    prediction = model.predict(preprocessed_img)
    
    if is_red(prediction[0]):  # 빨간색 감지 여부 확인
        print("빨간색 감지됨! 그리퍼 닫힘")
        mc.set_gripper_state(1, 20, 1)
        time.sleep(1)  # 그리퍼 상태 유지 시간 대기

        # 두 번째 각도로 이동
        print("두 번째 각도로 이동 중...")
        mc.send_angle(1, 120, 20)
        time.sleep(3)
        mc.send_angle(5, -90, 20)
        time.sleep(3)
        mc.send_angle(2, -80, 20)
        time.sleep(3)

        # 그리퍼 열기
        print("그리퍼 열림")
        mc.set_gripper_state(0, 20, 1)
        time.sleep(1)  # 그리퍼 상태 유지 시간 대기
    else:
        print("빨간색이 아님. 그리퍼를 닫지 않고 두 번째 각도로 이동하지 않음")

# 로봇 이동 및 그리퍼 제어 후 카메라 캡처
move_and_grip()
