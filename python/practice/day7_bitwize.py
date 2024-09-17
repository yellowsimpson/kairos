import cv2
import numpy as np

def nothing(x):
    pass


cap = cv2.VideoCapture(0)

# 이미지 로드
image = cv2.imread(cap)

cv2.namedWindow('Color Detector', cv2.WINDOW_NORMAL)

# Trackbar를 생성하여 'Lower'와 'Upper' 범위 설정
cv2.createTrackbar('Low H', 'Color Detector', 0, 179, nothing)
cv2.createTrackbar('High H', 'Color Detector', 179, 179, nothing)
cv2.createTrackbar('Low S', 'Color Detector', 0, 255, nothing)
cv2.createTrackbar('High S', 'Color Detector', 255, 255, nothing)
cv2.createTrackbar('Low V', 'Color Detector', 255, 255, nothing)
cv2.createTrackbar('High V', 'Color Detector', 255, 255, nothing)

while True:
    
    # 복사본 이미지 생성
    img = image.copy()

    _, frame =cap.read()   

    cv2.imshow("Video", frame )

    # 트랙바에서 현재 값 얻기
    low_h = cv2.getTrackbarPos('Low H', 'Color Detector')
    high_h = cv2.getTrackbarPos('High H', 'Color Detector')
    low_s = cv2.getTrackbarPos('Low S', 'Color Detector')
    high_s = cv2.getTrackbarPos('High S', 'Color Detector')
    low_v = cv2.getTrackbarPos('Low V', 'Color Detector')
    high_v = cv2.getTrackbarPos('High V', 'Color Detector')
    
    # HSV 색상 공간으로 이미지 변환
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
    
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
        if cv2.contourArea(max_contour) > 1000:
            cv2.drawContours(image, [max_contour], -1, (0, 255,0), 2)
        

    # 원본 이미지에 마스크 적용
    result = cv2.bitwise_and(img, img, mask=mask)

    # 결과 표시
    cv2.imshow('Color Detecter', result)
    cv2.imshow('Color Detector', image)
    
    # 키 입력 대기 (ESC를 누르면 종료)
    
    if cv2.waitKey(10) & 0xff== ord('q'):
        break
    # '''
    # hsv = [lower_color, upper_color]
    # type(hsv)
    # np.save("file_name", result)

    # if cv2.waitKey(10):
    #     if 0xff == ord('q'):
    #         break
    #     elif 0xff == ord('s'):
    #         np.save("file_name", hsv)
    # '''

cv2.destroyAllWindows()
