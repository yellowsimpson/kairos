import cv2
import numpy as np
from pymycobot.mycobot import MyCobot
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import img_to_array

# MyCobot 설정
mc = MyCobot('COM5', 115200)
mc.init_gripper()
mc.set_gripper_calibration()

# 학습된 모델 로드 (예: 'color_detection_model.h5')
model = load_model('color.h5')

def is_red(prediction):
    """예측 결과에서 빨간색이 포함되어 있는지 확인"""
    # 예측 결과가 [R, G, B] 형식으로 나왔다고 가정
    # 이 경우 R(빨간색) 채널 값이 다른 채널보다 높으면 빨간색으로 판별
    return prediction[0] > max(prediction[1], prediction[2])

def preprocess_image(img_path):
    """이미지를 모델에 맞게 전처리"""
    img = cv2.imread(img_path)
    img = cv2.resize(img, (224, 224))  # 학습 시 사용한 이미지 크기에 맞추기
    img = img_to_array(img)
    img = np.expand_dims(img, axis=0)
    img = img / 255.0  # 정규화 (모델 학습 시 적용된 정규화와 일치하게)
    return img

def capture_image():
    """카메라로부터 이미지 캡처"""
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    if ret:
        cv2.imwrite('frame.jpg', frame)
    cap.release()
    return 'frame.jpg'

while True:
    img_path = capture_image()
    preprocessed_img = preprocess_image(img_path)
    
    # 예측 수행
    prediction = model.predict(preprocessed_img)
    
    if is_red(prediction[0]):  # 예측 결과 첫 번째 값을 사용
        print("빨간색 감지됨! 그리퍼 닫힘")
        mc.set_gripper_state(0, 20, 1)
    else:
        print("빨간색이 아님. 그리퍼 열림")
        mc.set_gripper_state(1, 20, 1)
