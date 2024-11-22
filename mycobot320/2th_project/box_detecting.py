import cv2
import numpy as np

# 외부 USB 카메라 열기 (인덱스 1 사용)
cap = cv2.VideoCapture(1)

# 카메라가 정상적으로 열렸는지 확인
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    # 카메라에서 프레임 읽기
    ret, frame = cap.read()
    if not ret: 
        print("프레임을 가져올 수 없습니다.")
        break

    # 이미지를 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 블러링을 사용하여 노이즈 제거
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 엣지 감지를 위한 Canny 함수 사용
    edges = cv2.Canny(blurred, 50, 150)

    # 외곽선 찾기
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 각 외곽선에 대해 박스 그리기 및 중앙 점 찍기
    for contour in contours:
        if cv2.contourArea(contour) > 100:  # 작은 노이즈 필터링
            # 외곽선을 감싸는 직사각형 박스 계산
            x, y, w, h = cv2.boundingRect(contour)

            # 박스의 중앙 좌표 계산
            center_x = x + w // 2
            center_y = y + h // 2

            # 박스 그리기 (파란색)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # 중앙에 점 찍기 (빨간색)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    # 결과 프레임을 화면에 표시
    cv2.imshow('Real-time Box Detection with Center Point', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 웹캠과 윈도우 해제
cap.release()
cv2.destroyAllWindows()
