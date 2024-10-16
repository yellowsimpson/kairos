from pymycobot.mycobot import MyCobot
import time

# MyCobot 연결 설정
mc = MyCobot('COM6', 115200)

# 그리퍼 모드 설정 및 초기화
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
time.sleep(3)

# 3번 반복하는 루프
for _ in range(2):
    # 초기값: 모든 관절 각도 0으로 설정
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(3)  # 관절 이동을 위해 대기

    # 컨베이어 밸트 약잡는 위치의 각도로 이동 
    mc.send_angles([72.07, 41.66, 69.02, -28.53, -83.93, -22.76], 20)
    time.sleep(3)  # 관절 이동 후 대기

    # # 그리퍼 닫기 
    # mc.set_eletric_gripper(1)  # 그리퍼 닫기 명령
    # mc.set_gripper_value(80, 20, 1)  # 그리퍼를 완전히 닫음
    # time.sleep(5)  # 그리퍼 동작을 충분히 기다림

    # # 다시 모든 각도를 0으로 설정 (초기 위치로 복귀)
    # mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    # time.sleep(3)  # 관절 이동을 위해 대기

    # # 그리퍼 열기
    # mc.set_eletric_gripper(0)  # 그리퍼 열기 명령
    # mc.set_gripper_value(10, 20, 1)  # 그리퍼를 완전히 염 --> 그리퍼 를 여는 것이 아니라 그리퍼 각도를 10도 만드는 것 그래서 그래퍼가 열리지 않음
    # time.sleep(5)  # 그리퍼 동작을 충분히 기다림

    # mc.set_eletric_gripper(1) # 그리퍼 닫기 명령
    # mc.set_gripper_value(100, 20, 1) # 그리퍼 닫기 세부 설정 (각도, 속도, 1은 고정) 
    # time.sleep(3)

    mc.set_gripper_state(1,20,1) # 그리퍼 닫기 명령
    time.sleep(5)

    # A약통 각도로 이동
    mc.send_angles([10.19, 72.24, -19.16, 0.61, -80.06, 0.08], 20)
    time.sleep(8)  # 관절 이동 후 대기

    # # 지정된 각도로 이동
    # mc.send_angles([-149.76, -51.24, -30.93, -10.45, 89.92, 29.97], 20)
    # time.sleep(8)  # 관절 이동 후 대기

    mc.set_gripper_state(0,20,1) # 그리퍼 열기 명령
    time.sleep(5)

    mc.send_angles([0,0,0,0,0,0],20) # 그리퍼 위치 초기화
    time.sleep(3)



