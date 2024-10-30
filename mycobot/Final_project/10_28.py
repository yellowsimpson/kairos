  
# #pose1
# mc.send_angles([-135.35 , 77, 19.95, -14.41, -90, 46.86], 20)

# #pose2
# mc.send_angles([-135.35 , 40, 19.95, -14.41, -90, 46.86], 20)

# #pose3
# mc.send_angles([150, 40, 25, -14.41 , -90, 60], 20)

# #pose4(green)
# mc.send_angles([155, 50, 0, 7, -90, 75], 20) +

# #pose4(red)
# mc.send_angles([150, 70, 25, -14.41 , -90, 60], 20)

# #pose4(yellow)
# mc.send_angles([-41, -24, -76, 11, 90, 49], 20)

# #pose4(blue)
# mc.send_angles([-65, -20, -80, 70, 90, 30], 20)

from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 로봇 팔 각도 이동 후 카메라로 빨간색 탐지 및 그리퍼 동작
def perform_action_and_detect_red():
    print("로봇 팔 각도 이동")
    
    # 로봇 팔 각도 설정
    mc.send_angles([-135.35 , 0, 19.95, -14.41  , -90, 46.86], 20)
    time.sleep(3)
    mc.send_angles([-135.35 , 77, 19.95, -14.41, -90, 46.86], 20)
    time.sleep(3)
    

    # 카메라 초기화
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # 빨간색의 HSV 범위 설정
    lower_red_1 = np.array([0, 100, 100])   # 빨간색 하한 (범위 1)
    upper_red_1 = np.array([10, 255, 255])  # 빨간색 상한 (범위 1)

    lower_red_2 = np.array([170, 100, 100]) # 빨간색 하한 (범위 2)
    upper_red_2 = np.array([180, 255, 255]) # 빨간색 상한 (범위 2)

    # 탐지되지 않은 시간 초기화
    start_time = time.time()

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

        if len(contours) == 0:
            print("빨간색 탐지 안됨!!")  # 빨간색을 탐지하지 못할 때 출력
        else:
            # 빨간색이 탐지되었을 경우 타이머 리셋
            start_time = time.time()

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

                    # 빨간색 물체가 감지되면 그리퍼 닫기
                    print("빨간색 물체 감지! 그리퍼를 닫습니다.")
                    mc.set_gripper_state(1, 20, 1)  # 그리퍼 닫기
                    time.sleep(1)

                    # 카메라 해제 및 창 닫기
                    cap.release()
                    cv2.destroyAllWindows()

                    # pose 1
                    mc.send_angles([-135.35 , 40, 19.95, -14.41, -90, 46.86], 20)
                    time.sleep(3)

                    # pose 2
                    mc.send_angles([-135.35 , 40, 19.95, -14.41, -90, 46.86], 20)
                    time.sleep(3)
                    
                    # pose 3
                    mc.send_angles([150, 70, 25, -14.41 , -90, 60], 20)
                    time.sleep(3)

                    mc.set_gripper_state(0,20,1)
                    time.sleep(3)

                    mc.send_angles([0,0,0,0,0,0],20)
                    time.sleep(3)

                    return  # 작업이 완료되면 함수 종료

        # 빨간색을 10초 동안 탐지하지 못하면 초기 위치로 돌아감
        if time.time() - start_time > 10:
            print("10초 동안 빨간색이 탐지되지 않았습니다. 초기 위치로 돌아갑니다.")
            mc.send_angles([0, 0, 0, 0, 0, 0], 20)
            break

        # 결과 이미지와 원본 이미지에 윤곽선 표시
        cv2.imshow('Webcam', frame)
        cv2.imshow('Red Color Detection', red_result)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

# 메인 실행
perform_action_and_detect_red()
