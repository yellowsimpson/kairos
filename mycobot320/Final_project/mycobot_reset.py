from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6', 115200)

mc.set_gripper_mode(0)

#로봇암의 위치 초기화
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# mc.send_angles([1번 모터 각도, 2번 모터 각도, 3번 모터 각도, 4번 모터 각도, 5번 모터 각도, 6번 모터 각도], 모터 속도)
time.sleep(2)

#그리퍼 열고 닫기
mc.set_gripper_state(0, 100)
#mc.set_gripper_state(0 : 그리퍼 열기, 1 : 그리퍼 닫기, 모터 속도)
time.sleep(2)

#그리퍼 범위설정해서 열고, 닫기
# mc.set_gripper_value(100,40)
# mc.set_gripper_value(그리퍼의 닫힘 범위 0 ~ 100에서 설정 , 모터 속도)
# time.sleep(2)0