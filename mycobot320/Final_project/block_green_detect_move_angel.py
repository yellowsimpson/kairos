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

# 로봇 팔 각도 이동 후 카메라로 초록색 탐지 및 그리퍼 동작
def perform_action_and_detect_green():
    print("로봇 팔 각도 이동")
    
    # 로봇 팔 각도 설정
    mc.send_angles([72.07, 41.66, 69.02, -28.53, -83.93, -22.76], 20)
    time.sleep(3)

    # 카메라 초기화
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # 초록색의 HSV 범위 설정
    lower_green = np.array([35, 100, 100])  # 초록색 하한
    upper_green = np.array([85, 255, 255])  # 초록색 상한

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
        
        # 초록색 범위 내에 있는 모든 픽셀을 찾아 마스크 생성
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

        # 마스크를 사용해 원본 이미지에서 초록색 부분만 추출
        green_result = cv2.bitwise_and(frame, frame, mask=green_mask)

        # 윤곽선을 찾아 화면에 표시
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) == 0:
            print("초록색 탐지 안됨!!")  # 초록색을 탐지하지 못할 때 출력
        else:
            # 초록색이 탐지되었을 경우 타이머 리셋
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

                    # 초록색 물체가 감지되면 그리퍼 닫기
                    print("초록색 물체 감지! 그리퍼를 닫습니다.")
                    mc.set_gripper_state(1, 20, 1)  # 그리퍼 닫기
                    time.sleep(1)

                    # 카메라 해제 및 창 닫기
                    cap.release()
                    cv2.destroyAllWindows()

                    # pose 1
                    mc.send_angles([72.07, 20.66, 69.02, -28.53, -83.93, -22.76], 20)
                    time.sleep(3)

                    # pose 2
                    mc.send_angles([-19, 20, 1, 12, -90, -22], 20)
                    time.sleep(3)
                    
                    # pose 3
                    mc.send_angles([-19, 76, 1, 12, -90, -22], 20)
                    time.sleep(3)

                    mc.set_gripper_state(0,20,1)
                    time.sleep(3)

                    mc.send_angles([0,0,0,0,0,0],20)
                    time.sleep(3)

                    return  # 작업이 완료되면 함수 종료

        # 초록색을 10초 동안 탐지하지 못하면 초기 위치로 돌아감
        if time.time() - start_time > 10:
            print("10초 동안 초록색이 탐지되지 않았습니다. 초기 위치로 돌아갑니다.")
            mc.send_angles([0, 0, 0, 0, 0, 0], 20)
            break

        # 결과 이미지와 원본 이미지에 윤곽선 표시
        cv2.imshow('Webcam', frame)
        cv2.imshow('Green Color Detection', green_result)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

# 메인 실행
perform_action_and_detect_green()
