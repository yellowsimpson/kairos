import cv2
import time

# 카메라를 초기화합니다 (0번 장치는 기본 카메라)
cap = cv2.VideoCapture(1)

# 카메라가 정상적으로 열렸는지 확인합니다
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 연속 캡처 시작
num_images = 200  # 캡처할 이미지 수
delay = 0.1  # 각 프레임 사이의 지연 시간 (초)
output_folder = 'captured_red/'  # 이미지 저장 폴더

# 저장 폴더가 없으면 생성합니다
import os
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

for i in range(num_images):
    # 프레임을 캡처합니다
    ret, frame = cap.read()

    if ret:
        # 이미지를 파일로 저장합니다
        filename = f'{output_folder}image_{i:03d}.jpg'
        cv2.imwrite(filename, frame)
        print(f"{filename} 저장 완료")
    else:
        print("이미지를 캡처할 수 없습니다.")
        break

    # 지연 시간을 적용합니다
    time.sleep(delay)

# 카메라를 닫습니다
cap.release()
cv2.destroyAllWindows()
