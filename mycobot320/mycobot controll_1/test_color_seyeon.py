import cv2
import numpy as np
from tensorflow.keras.models import load_model
from pymycobot.mycobot import MyCobot
import time

# 모델 로드 (robot_color.h5)
model = load_model('robot_color.h5')

# 클래스 이름 (예시: 빨간색, 파란색, 초록색을 탐지한다고 가정)
class_names = ['Red', 'Green', 'Blue']

def predict_color(frame):
    """모델을 사용하여 이미지의 색상 예측"""
    # 이미지를 224x224로 리사이즈 (모델이 요구하는 크기)
    resized_frame = cv2.resize(frame, (224, 224))  # 모델 입력 크기로 리사이즈
    img_array = np.expand_dims(resized_frame, axis=0)  # 배치 차원을 추가 (1, 224, 224, 3)
    
    # 모델로 예측
    predictions = model.predict(img_array)
    predicted_class = np.argmax(predictions, axis=1)  # 예측된 클래스 인덱스
    predicted_color = class_names[predicted_class[0]]  # 예측된 색상 이름
    return predicted_color

class MyCobotController:
    def __init__(self):
        # MyCobot 연결 설정
        self.mc = MyCobot('COM13', 115200)
        
        # 시작 위치 각도
        self.start_angles = [-8, 47, 19, 23, -87, -15]

        # 물체를 잡는 위치
        self.grab_angle = [-6, 44, 39, -12, -94, 0]
        
        # 빨간색 물체 위치에서 놓는 각도
        self.grab_red = [27, 72, 0, -21, -69, -15]
        
        # 파란색 물체 위치에서 놓는 각도
        self.grab_blue = [45, 60, 6, 0, -72, -33]
        
        # 초록색 물체 위치에서 놓는 각도
        self.grab_green = [66, 63, -6, 15, -75, 0]

        # 중간 위치 각도 (0, 0, 0, 0, 0, 0)
        self.mid_position = [0, 0, 0, 0, 0, 0]

    def move_to_start(self):
        """로봇을 시작 위치로 이동"""
        for i, angle in enumerate(self.start_angles):
            self.mc.send_angle(i + 1, angle, 20)
            time.sleep(1)
        print("Moved to start position.")

    def move_to_grab(self):
        """물체를 잡기 위한 위치로 이동"""
        print("Moving to grab position.")
        for i, angle in enumerate(self.grab_angle):
            self.mc.send_angle(i + 1, angle, 20)
            time.sleep(1)

        # 그리퍼 닫기 (물체 잡기)
        self.mc.set_gripper_state(1, 50)  # 그리퍼 닫기
        time.sleep(2)  # 물체를 잡을 시간 대기
        print("Object grabbed.")

    def move_to_position(self, target_angles):
        """로봇을 특정 위치로 이동하고 물체를 놓기"""
        # 중간 위치로 이동
        self.move_to_mid_position()

        # 지정된 위치로 이동
        print("Moving to specified position.")
        for i, angle in enumerate(target_angles):
            self.mc.send_angle(i + 1, angle, 20)
            time.sleep(1)

        # 그리퍼 열기 (물체 놓기)
        self.mc.set_gripper_state(0, 50)  # 그리퍼 열기
        print("Object released.")

    def move_to_mid_position(self):
        """중간 위치 (0, 0, 0, 0, 0, 0)로 이동"""
        print("Moving to mid position (0, 0, 0, 0, 0, 0).")
        for i, angle in enumerate(self.mid_position):
            self.mc.send_angle(i + 1, angle, 20)
            time.sleep(1)
        print("Moved to mid position.")

    def check_for_color_object(self):
        """카메라를 켜서 물체의 색상을 인식"""
        cap = cv2.VideoCapture(1)  # 카메라 시작
        if not cap.isOpened():
            print("Error: Could not open camera.")
            return None

        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            cap.release()
            return None

        # 색상 예측
        predicted_color = predict_color(frame)
        print(f"Predicted color: {predicted_color}")

        # 카메라 해제
        cap.release()

        return predicted_color

    def run(self):
        """메인 루프: 시작 위치 -> 카메라로 물체 색상 확인 -> 색상별로 물체를 잡고 놓기 -> 다시 시작 위치"""
        while True:
            # 시작 위치로 이동
            self.move_to_start()

            # 카메라로 물체의 색상을 탐지
            print("Checking for object color...")
            detected_color = self.check_for_color_object()

            if detected_color:
                print(f"{detected_color} object detected!")
                
                # 물체를 잡는 위치로 이동해서 잡음
                self.move_to_grab()

                # 색상에 따라 물체를 놓는 위치로 이동
                if detected_color == 'Red':
                    self.move_to_position(self.grab_red)  # 빨간색 물체를 지정된 위치로 옮김
                elif detected_color == 'Blue':
                    self.move_to_position(self.grab_blue)  # 파란색 물체를 지정된 위치로 옮김
                elif detected_color == 'Green':
                    self.move_to_position(self.grab_green)  # 초록색 물체를 지정된 위치로 옮김

            else:
                print("No recognized object detected.")
            
            time.sleep(2)  # 다음 탐지 주기 전 대기 시간

if __name__ == '__main__':
    # MyCobot 컨트롤러 생성
    controller = MyCobotController()

    # 메인 루프 실행
    controller.run()