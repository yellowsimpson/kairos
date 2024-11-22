from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('/dev/ttyACM0', 115200)

# YOLO 모델 로드
model = YOLO('/home/shim/github/KG_2_Project/ROOBOTARM_team/yolov8_model/runs/detect/train2/weights/best.pt')

# 픽셀-로봇 좌표 변환 비율 설정
pixel_to_robot_x = 0.2  # X축 변환 비율
pixel_to_robot_y = 0.2  # Y축 변환 비율
    
# pose2 위치(Coordination Control) 설정 (z축과 회전값 고정)
pose2_coords = [30.9, -327.5, 261.9, -170.94, 0.15, 170]
fixed_z = pose2_coords[2]  # z 축 고정

# Z축을 내릴 위치 설정
lowered_z = fixed_z - 100  # 원하는 만큼 z축을 내립니다 (단위: mm)

# 초기 위치 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False  # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

# 웹캠 설정
cap = cv2.VideoCapture(2)
CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "YOLO Detection View"

# HSV 블록 색상 확인 함수
def detect_color():
    # HSV 색상 범위 설정
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([170, 100, 100])
    upper_red_2 = np.array([180, 255, 255])

    # 카메라로부터 색상 탐지
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        red_mask1 = cv2.inRange(hsv_frame, lower_red_1, upper_red_1)
        red_mask2 = cv2.inRange(hsv_frame, lower_red_2, upper_red_2)
        red_mask = cv2.add(red_mask1, red_mask2)

        if cv2.countNonZero(blue_mask) > 0:
            print("파란색 블록 탐지!")
            return 'blue'
        elif cv2.countNonZero(yellow_mask) > 0:
            print("노란색 블록 탐지!")
            return 'yellow'
        elif cv2.countNonZero(green_mask) > 0:
            print("초록색 블록 탐지!")
            return 'green'
        elif cv2.countNonZero(red_mask) > 0:
            print("빨간색 블록 탐지!")
            return 'red'

        if time.time() - start_time > 10:
            print("10초 동안 색상이 감지되지 않았습니다.")
            return None

last_detected_color = None  # 전역 변수로 블록 색상 기억

# pose0에서 블록 색상 확인 및 잡기
def detect_and_grab_block():
    global last_detected_color
    time.sleep(3)
    mc.send_angles([-15, 65, 16, 0, -90, 0], 20)  # pose0_1 웹캠으로 블록 확인 위치
    time.sleep(8)
    
    # 그리퍼 초기화 및 캘리브레이션
    mc.set_gripper_mode(0)
    mc.init_gripper()
    mc.set_gripper_calibration()
    time.sleep(2)

    detected_color = detect_color()
    if detected_color:
        last_detected_color = detected_color  # 블록 색상 기억
        print(f"탐지된 색상: {detected_color}")

        # 초록색 블록(불량품)일 때 예외 처리
        if detected_color == 'green':
            print("초록색 블록이 감지되었습니다. 지정된 위치로 이동 후 복귀합니다.")
            mc.send_angles([-15, 74, 11, 0, -90, 0], 20)  # pose0_2 그리퍼로 블록 잡는 위치
            time.sleep(3)
            mc.set_gripper_state(1, 20, 1)  # 그리퍼 닫기
            time.sleep(3)
            mc.send_angles([-15, 30, 11, 0, -90, 0], 20)  # pose1
            time.sleep(3)
            mc.send_angles([-90, 60, 11, 22, -90, 0], 20)  # 초록색 블록 놓을 위치로 이동
            time.sleep(7)
            mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
            time.sleep(5)
            mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # pose3
            time.sleep(5)
            print("초록색 블록 작업 완료. 복귀 중...")
            return True  # 예외 동작을 수행했음을 알림

        # 다른 색상일 경우 블록 잡기 동작 수행
        mc.send_angles([-15, 74, 11, 0, -90, 0], 20)  # pose0_2 그리퍼로 블록 잡는 위치
        time.sleep(3)
        mc.set_gripper_state(1, 20, 1)  # 그리퍼로 블록 잡기
        time.sleep(2)
        print("블록을 성공적으로 잡았습니다!")
        return False
    else:
        print("블록의 색상을 감지하지 못했습니다.")
        return False

# pose2에서 객체 인식 후 중점(빨간 점)을 인식하고 조정
def perform_pose2_adjustments():
    global centered
    centered = False  # 조정 시작 시 centered 초기화
    mc.send_angles([80, 50, 11, 22, -90, 0], 20)  # pose2
    time.sleep(5)

    # 중심 맞추기 시작 시간 기록
    start_time = time.time()
    time_limit = 10  # 10초 제한

    # 빨간 점을 화면 중앙으로 맞출 때까지 조정 (최대 10초 동안)
    while not centered and (time.time() - start_time) < time_limit:
        detect_and_adjust_position()

    # 10초 경과 후 또는 중심 맞추기 완료 후 처리
    if not centered:
        print(f"{time_limit}초 내에 현재 위치에서 로봇 암을 내립니다.")
    else:
        print("빨간 점이 목표 좌표 근처에 위치했습니다.")

def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)

def detect_and_adjust_position():
    global current_x, current_y, centered, first_detection
    ret, frame = cap.read()
    if not ret:
        print("카메라에서 프레임을 가져올 수 없습니다.")
        return

    # YOLO 모델 적용
    results = model(frame)
    frame_with_yolo = results[0].plot()

    # 보정 비율 설정 (픽셀 -> 로봇 좌표 변환)
    pixel_to_robot_x = 0.5  # 픽셀 이동에 따른 x축 보정 비율
    pixel_to_robot_y = 0.5  # 픽셀 이동에 따른 y축 보정 비율

    # 빨간 점 인식 후 조정
    for result in results:
        for box in result.boxes:
            if box.conf >= CONFIDENCE_THRESHOLD:
                # 바운딩 박스의 중심 계산
                x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
                x_center = (x_min + x_max) // 2
                y_center = (y_min + y_max) // 2

                cv2.circle(frame_with_yolo, (x_center, y_center), 5, (0, 0, 255), -1)

                # 중심점 위치 출력
                if first_detection:
                    print(f"중심점 위치(X좌표 , Y좌표) : ({x_center}, {y_center})")
                    first_detection = False

                # 부호 반대로 적용하여 조정값 계산
                adjust_x = (TARGET_X - x_center) * pixel_to_robot_x * (1)  # X축 부호 반전
                adjust_y = (TARGET_Y - y_center) * pixel_to_robot_y * (-1)  # Y축 부호 반전
                print(f"조정값(X좌표 , Y좌표) : ({adjust_x}, {adjust_y})")

                # 새로운 x, y 값을 계산하고 업데이트
                current_x = pose2_coords[0] + adjust_x
                current_y = pose2_coords[1] + adjust_y

                # 로봇을 조정된 좌표로 이동
                move_to_position(current_x, current_y, fixed_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

                # 중심이 목표 좌표에 근접했는지 확인
                if abs(x_center - TARGET_X) < 10 and abs(y_center - TARGET_Y) < 10:
                    centered = True
                    print("빨간 점이 목표 좌표 근처에 위치했습니다.")
                    return

    # 실시간으로 YOLO Detection View 업데이트
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 600, 600)
    cv2.imshow(WINDOW_NAME, frame_with_yolo)

    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
        cap.release()
        cv2.destroyAllWindows()
        exit()

def block_box_match():
    x, y = current_x, current_y
    # 현재 로봇 암의 Z축 위치를 가져옵니다 (로봇 암이 내려간 위치)
    z = mc.get_coords()[2]
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    if last_detected_color == 'blue':
        y += 120
        print("파란색블록입니다. 왼쪽으로 120 픽셀만큼 이동합니다.")
    elif last_detected_color == 'yellow':
        y -= 0
        print("노란색블록입니다. 블록을 내려놓습니다.")
    elif last_detected_color == 'red':
        y -= 120
        print("빨간색블록입니다. 오른쪽으로 120픽셀만큼 이동합니다.")

    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    move_to_position(x, y, z, rx, ry, rz)
    time.sleep(5)

def lower_z():
    global current_x, current_y, lowered_z
    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])
    time.sleep(5)  # 이동 시간 대기

def main():
    # detect_and_grab_block() 호출 시 초록색 블록을 감지하면 True를 반환
    green_detected = detect_and_grab_block()
    
    # 초록색 블록을 감지하지 않은 경우에만 이후 동작 수행
    if not green_detected:
        mc.send_angles([-15, 30, 11, 0, -90, 0], 20)  # pose1
        time.sleep(5)
        perform_pose2_adjustments()  # pose2로 이동 후 빨간 점 인식 및 조정
        time.sleep(5)

        lower_z()
        time.sleep(3)

        block_box_match()  # 블록 색상에 따른 위치 조정
        time.sleep(3)

        print("그리퍼가 열립니다.")
        mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
        time.sleep(3)

        mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # pose3 작업 완료 후 돌아가기
        time.sleep(7)
        print("모든 작업을 완료하고 복귀합니다.")

for i in range(4):
    main()
    print(f"{i+1}번째 작업 완료")
