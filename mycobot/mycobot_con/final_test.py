from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from tensorflow.keras.models import load_model

# MyCobot 초기화
mc = MyCobot('COM5', 115200)

# 그리퍼 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_state(0, 20, 1)
mc.set_gripper_calibration()
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
time.sleep(3)

# 첫 번째 각도로 이동
print('첫번째 각도로 이동')
mc.send_angle(1, 90, 20)
time.sleep(3)
mc.send_angle(5, -90, 20)
time.sleep(3)
mc.send_angle(2, 80, 20)
time.sleep(3)

# 카메라 켜기
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 학습된 모델 불러오기
model = load_model('colorblock2.h5')

# 프레임을 받아 색 검출
def detect_red(frame):
    # 이미지 전처리
    frame = cv2.resize(frame, (64, 64))
    frame = frame.astype('float32') / 255.0
    frame = np.expand_dims(frame, axis=0)
    
    # 모델 예측
    prediction = model.predict(frame)
    return np.argmax(prediction) == 1  # 빨간색이 클래스 '1'일 때

# 빨간색 검출 후 그리퍼 닫기
while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 가져올 수 없습니다.")
        break
    
    cv2.imshow("카메라 화면", frame)  # 카메라 화면 표시

    if detect_red(frame):
        print("빨간색 검출됨, 그리퍼 닫기")
        mc.set_gripper_state(1, 20, 1)
        time.sleep(3)
        break

    # 'q' 키를 눌러 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 두 번째 각도로 이동
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
time.sleep(3)
mc.send_angle(5, -90, 20)
time.sleep(3)
mc.send_angle(2, 80, 20)
time.sleep(3)

# 그리퍼 열기
mc.set_gripper_state(0, 20, 1)
time.sleep(3)

# 로봇팔 리셋
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
