from pymycobot.mycobot import MyCobot

mc = MyCobot('COM6',115200)
mc.set_gripper_mode(0)
mc.init_gripper()
mc.set_gripper_calibration()
mc.set_gripper_state(0,20,1) # 그리퍼 열림
mc.send_angles([0,0,0,0,0,0],20)