import cv2
import time
import os

# 사진을 저장할 디렉토리 설정
output_dir = "captured_images3"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 카메라 초기화
cap = cv2.VideoCapture(1)  # 0은 기본 카메라 장치입니다. USB 카메라가 기본 장치로 설정된 경우

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 이미지 촬영 및 저장
num_images = 200
interval = 0.5  # 초 단위s

for i in range(num_images):
    # 프레임 캡처
    ret, frame = cap.read()
    if not ret:
        print(f"{i+1}번째 이미지 캡처 실패")
        continue

    # 이미지 저장
    image_path = os.path.join(output_dir, f"image_{i+1:03d}.jpg")
    cv2.imwrite(image_path, frame)
    print(f"이미지 {i+1} 저장 완료: {image_path}")

    # 0.5초 대기
    time.sleep(interval)

# 카메라 해제
cap.release()
print("모든 이미지 캡처 완료")

# 모든 윈도우 닫기
cv2.destroyAllWindows()
