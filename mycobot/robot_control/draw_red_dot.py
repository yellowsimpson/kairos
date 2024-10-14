# -*- coding:utf-8 -*-
import cv2
import numpy as np
# 카메라 초기화
cap = cv2.VideoCapture(0)
cap.set(3, 320)  # Width 설정
cap.set(4, 240)  # Height 설정
while True:
    ret, img = cap.read()
    if not ret:
        break  # 비디오 읽기 실패 시 종료
    # 1. 이미지 색상 공간 변환 (BGR -> HSV)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 2. 빨간색 마스크 생성 (HSV 두 범위)
    lower_red1 = np.array([0, 150, 150])   # 더 높은 채도와 명도로 설정
    upper_red1 = np.array([10, 255, 255])  # 첫 번째 빨간색 범위
    lower_red2 = np.array([170, 150, 150]) # 두 번째 빨간색 범위
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
    red_mask = mask1 + mask2  # 두 마스크를 합쳐 하나의 마스크로
    # 3. 노이즈 제거를 위한 모폴로지 연산 (열림 연산: 작은 노이즈 제거)
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    # 4. 마스크를 이용해 빨간색 영역만 남기고 나머지는 블랙 처리
    red_only = cv2.bitwise_and(img, img, mask=red_mask)
    # 5. 외곽선 탐지 (빨간색 영역의 경계 검출)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    try:
        # 가장 큰 외곽선을 찾아 중심 계산
        c = max(contours, key=cv2.contourArea)
        # 중심 좌표 계산 (모멘트 이용)
        M = cv2.moments(c)
        if M['m00'] != 0:  # 분모가 0이 아닌지 확인
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            # 중심에 초록색 원 표시
            red_only = cv2.circle(red_only, (cx, cy), 10, (0, 255, 0), -1)
        # 외곽선 그리기 (빨간색으로 표시)
        red_only = cv2.drawContours(red_only, [c], -1, (0, 0, 255), 2)
    except ValueError:
        pass  # 예외 처리: 외곽선이 없을 경우
    # 6. 결과 이미지 출력
    cv2.imshow('Red Area Detection', red_only)
    cv2.imshow('mask', red_mask)
    # 종료 조건: 'q' 키를 누르면 종료
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
# 자원 해제
cap.release()
cv2.destroyAllWindows()