from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np

# MyCobot 초기화
mc = MyCobot('COM5', 115200)

# 그리퍼 설정
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
mc.set_gripper_state(1, 20, 1)  # 그리퍼를 닫기
time.sleep(1)

# 로봇의 각도 설정
mc.send_angle(1, 80, 20)
time.sleep(3)
mc.send_angle(5, -90, 20)
time.sleep(3)
# 그리퍼 열기
mc.set_gripper_state(0, 20, 1)  # 그리퍼를 열기
time.sleep(1)
mc.send_angle(2, 80, 20)
time.sleep(3)



# 카메라 초기화
cap = cv2.VideoCapture(1)  # 1번 카메라 장치 사용

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # HSV 색상 공간으로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 빨간색 범위 정의 (조정된 범위)
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # 빨간색 영역 추출
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    # 빨간색 영역의 면적 계산
    red_area = np.sum(mask > 300)

    # 빨간색 객체가 검출되면 그리퍼를 닫고, 각도를 0으로 설정한 후, 그리퍼를 다시 열기
    if red_area > 1000:  # 빨간색 영역의 면적 기준 설정
        mc.set_gripper_state(1, 20, 1)  # 그리퍼를 닫기
        print("빨간색 객체를 감지하여 그리퍼를 닫았습니다.")
        time.sleep(3)  # 그리퍼 동작 확인 대기

        # 모든 각도를 0으로 설정
        mc.send_angles([0, 0, 0, 0, 0, 0], 20)
        print("모든 각도를 0으로 설정했습니다.")
        time.sleep(3)  # 각도 이동 대기

        # 그리퍼를 다시 열기
        mc.set_gripper_state(0, 20, 1)  # 그리퍼를 열기
        print("그리퍼를 열었습니다.")
        time.sleep(3)  # 그리퍼 동작 확인 대기
        break    
    # 결과 출력
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("Original", frame)
    cv2.imshow("Red Detection", result)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()
