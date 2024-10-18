# from pymycobot.mycobot import MyCobot
# import time

# mc = MyCobot('COM6',115200)
# mc.set_gripper_mode(0)
# mc.init_gripper()
# mc.set_gripper_calibration()
# mc.set_gripper_state(0,20,1) # 그리퍼 열림
# mc.send_angles([0,0,0,0,0,0],20)


# from pymycobot.mycobot import MyCobot
# import time

# mc = MyCobot('COM6', 115200)

# mc.set_gripper_mode(0)

# mc.init_gripper()

# mc.set_gripper_calibration()

# mc.send_angles([0, 0, 0, 0, 0, 0], 20)
# time.sleep(2)

# mc.set_gripper_state(0, 20, 1)
# time.sleep(2)


from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6', 115200)

mc.set_gripper_mode(0)  # 설정 모드로 변경
mc.init_gripper()       # 그리퍼 초기화

time.sleep(2)
mc.set_gripper_state(1, 100, 1)  # 그리퍼 열기
time.sleep(2)
mc.set_gripper_state(0, 100, 1)  # 그리퍼 닫기
