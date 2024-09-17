import cv2
import numpy as np

def nothing(x):
    pass

# 카메라 로드
cap = cv2.VideoCapture(0)

cv2.namedWindow('Color Detector', cv2.WINDOW_NORMAL)

# Trackbar를 생성하여 'Lower'와 'Upper' 범위 설정
cv2.createTrackbar('Low H', 'Color Detector', 0, 179, nothing)
cv2.createTrackbar('High H', 'Color Detector', 179, 179, nothing)
cv2.createTrackbar('Low S', 'Color Detector', 0, 255, nothing)
cv2.createTrackbar('High S', 'Color Detector', 255, 255, nothing)
cv2.createTrackbar('Low V', 'Color Detector', 0, 255, nothing)
cv2.createTrackbar('High V', 'Color Detector', 255, 255, nothing)
# 면적 트랙바 추가
cv2.createTrackbar('Min Area', 'Color Detector', 500, 2000, nothing)
cv2.createTrackbar('Max Area', 'Color Detector', 1000, 5000, nothing)

while True:
    # 웹캠 프레임 읽기
    _, frame = cap.read()

    # 트랙바에서 현재 값 얻기
    low_h = cv2.getTrackbarPos('Low H', 'Color Detector')
    high_h = cv2.getTrackbarPos('High H', 'Color Detector')
    low_s = cv2.getTrackbarPos('Low S', 'Color Detector')
    high_s = cv2.getTrackbarPos('High S', 'Color Detector')
    low_v = cv2.getTrackbarPos('Low V', 'Color Detector')
    high_v = cv2.getTrackbarPos('High V', 'Color Detector')
    min_area = cv2.getTrackbarPos('Min Area', 'Color Detector')
    max_area = cv2.getTrackbarPos('Max Area', 'Color Detector')

    # HSV 색상 공간으로 이미지 변환
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # HSV에서의 lower/upper 임계값
    lower_color = np.array([low_h, low_s, low_v])
    upper_color = np.array([high_h, high_s, high_v])

    # lower/upper 임계값을 이용하여 마스크 생성
    mask = cv2.inRange(img_hsv, lower_color, upper_color)

    # 노이즈 제거를 위한 모폴로지 연산 적용
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 윤곽선 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        max_contour = max(contours, key=cv2.contourArea)
        if min_area < cv2.contourArea(max_contour) < max_area:
            cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)

    # 원본 이미지에 마스크 적용
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # 결과 표시
    cv2.imshow('Color Detector', result)

    # 키 입력 대기 (ESC를 누르면 종료)
    if cv2.waitKey(10) & 0xff == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
