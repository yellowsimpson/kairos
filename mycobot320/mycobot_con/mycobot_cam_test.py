import cv2
import numpy as np

# 웹캠을 캡처하는 객체 생성 (0번 카메라)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

# 빨간색의 HSV 범위 설정
lower_red_1 = np.array([0, 100, 100])   # 빨간색 하한 (범위 1)
upper_red_1 = np.array([10, 255, 255])  # 빨간색 상한 (범위 1)

lower_red_2 = np.array([170, 100, 100]) # 빨간색 하한 (범위 2)
upper_red_2 = np.array([180, 255, 255]) # 빨간색 상한 (범위 2)

while True:
    # 프레임을 읽어오기
    ret, frame = cap.read()
    
    if not ret:
        print("프레임을 수신할 수 없습니다. 스트림이 종료되었습니다.")
        break

    # BGR 이미지를 HSV로 변환
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 빨간색 범위 내에 있는 모든 픽셀을 찾아 마스크 생성 (두 범위 합침)
    mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
    red_mask = cv2.add(mask1, mask2)

    # 마스크를 사용해 원본 이미지에서 빨간색 부분만 추출
    red_result = cv2.bitwise_and(frame, frame, mask=red_mask)

    # 윤곽선을 찾아 화면에 표시
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # 너무 작은 윤곽선은 무시
        if cv2.contourArea(contour) > 500:
            # 윤곽선을 그리기
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            
            # 윤곽선의 중심에 원을 그리기
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

    # 결과 이미지와 원본 이미지에 윤곽선 표시
    cv2.imshow('Webcam', frame)
    cv2.imshow('Red Color Detection', red_result)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 모든 작업이 끝나면 캡처 객체와 창을 닫습니다.
cap.release()
cv2.destroyAllWindows()
