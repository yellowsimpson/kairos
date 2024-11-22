from pymycobot.mycobot import MyCobot
import time
import cv2
import numpy as np
from ultralytics import YOLO

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# YOLO 모델 로드
model = YOLO('C:\\Users\shims\\Desktop\\github\\KG_2_Project\\ROOBOTARM_team\\yolov8_model\\runs\\detect\\train2\\weights\\best.pt')

# 픽셀-로봇 좌표 변환 비율 설정
pixel_to_robot_x = 0.2  # X축 변환 비율
pixel_to_robot_y = 0.2  # Y축 변환 비율

# pose2 위치(Coordination Control) 설정 (z축과 회전값 고정)
pose2_coords = [30.9, -327.5, 261.9, -170.94, 0.15, 170]
fixed_z = pose2_coords[2]  # z축 고정

# Z축을 내릴 위치 설정
lowered_z = fixed_z - 150  # 원하는 만큼 z축을 내립니다 (단위: mm)

# 초기 위치 설정
current_x, current_y = pose2_coords[0], pose2_coords[1]
centered = False  # 중심 맞추기 완료 여부 확인
first_detection = True  # 처음 중심점 위치 출력 여부 확인

# 웹캠 설정 (전역 변수)
cap = cv2.VideoCapture(1)  # 전역 변수로 선언
CONFIDENCE_THRESHOLD = 0.7
TARGET_X, TARGET_Y = 300, 300
WINDOW_NAME = "QR Code Detection"

# QR 코드 인식 함수
def detect_qr_code(frame):
    detector = cv2.QRCodeDetector()
    data, points, _ = detector.detectAndDecode(frame)
    if points is not None and data:
        print(f"QR 코드 인식됨! 내용: {data}")
        return data
    return None

last_detected_qr = None  # 전역 변수로 QR 코드 기억

# pose0에서 QR 코드 인식 후 블록 잡기
def detect_and_grab_block():
    global last_detected_qr
    global cap  # 전역 변수 cap 사용
    time.sleep(3)
    mc.send_angles([-15, 65, 16, 0, -90, 0], 20)  # pose0_1 웹캠으로 블록 확인 위치
    time.sleep(5)

    # 그리퍼 초기화 및 캘리브레이션
    mc.set_gripper_mode(0)
    mc.init_gripper()
    mc.set_gripper_calibration()
    time.sleep(2)

    detected_qr = detect_qr_code()
    if detected_qr:
        last_detected_qr = detected_qr  # QR 코드 데이터 저장
        print(f"탐지된 QR 코드: {detected_qr}")

        # 블록 잡기
        mc.send_angles([-15, 74, 11, 0, -90, 0], 20)  # pose0_2 그리퍼로 블록 잡는 위치
        time.sleep(3)
        mc.set_gripper_state(1, 20, 1)  # 그리퍼로 블록 잡기
        time.sleep(3)

        print("블록을 성공적으로 잡았습니다!")
        return True
    else:
        print("QR 코드를 감지하지 못했습니다.")
        return False

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

# 특정 좌표로 이동
def move_to_position(x, y, z, rx, ry, rz, speed=20):
    print(f"Moving to position: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    mc.send_coords([x, y, z, rx, ry, rz], speed)
    time.sleep(5)

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

# QR 코드에 따라 블록 놓기
def block_box_match():
    x, y = current_x, current_y
    z = mc.get_coords()[2]  # 현재 z축 위치 가져오기
    rx, ry, rz = pose2_coords[3], pose2_coords[4], pose2_coords[5]

    if last_detected_qr == 'https://site.naver.com/patient/A_1':
        y += 120
        print("A_1 블록: 왼쪽 아래로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/A_2':
        print("A_2 블록: 중앙 아래로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/A_3':
        y -= 120
        print("A_3 블록: 오른쪽 아래로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/B_1':
        y += 120
        print("B_1 블록: 왼쪽 위로로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/B_2':
        print("B_2 블록: 중앙 위로로 이동합니다.")
    elif last_detected_qr == 'https://site.naver.com/patient/B_3':
        y -= 120
        print("B_3 블록: 오른쪽 위로 이동합니다.")
    
    print(f"블록을 놓는 위치로 이동: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
    move_to_position(x, y, z, rx, ry, rz)

def lower_z():
    global lowered_z
    print("로봇암을 아래로 내립니다.")
    move_to_position(current_x, current_y, lowered_z, pose2_coords[3], pose2_coords[4], pose2_coords[5])

def main():
    detect_and_grab_block()
    mc.send_angles([-15, 30, 11, 0, -90, 0], 20)  # pose1
    time.sleep(5)
    perform_pose2_adjustments()  # pose2로 이동 후 빨간 점 인식 및 조정
    time.sleep(5)
    lower_z()        # Z축 내리기
    time.sleep(5)
    block_box_match() # QR 코드에 따라 위치 이동
    time.sleep(5)

    print("그리퍼가 열립니다")
    mc.set_gripper_state(0, 20, 1)  # 그리퍼 열기
    time.sleep(3)

    mc.send_angles([0, 0, 0, 0, 0, 0], 20)  # 초기 위치
    time.sleep(5)
    print("모든 작업을 완료하고 복귀합니다.")

# 실행 루프
for i in range(4):
    main()
    print(f"{i+1}번째 작업 완료")
