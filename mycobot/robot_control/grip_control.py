from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM6',115200)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(1)
while True:
  mc.set_eletric_gripper(1)
  mc.set_gripper_value(100,20,1)
  time.sleep(2)
  mc.set_eletric_gripper(0)
  mc.set_gripper_value(0,20,1)
  time.sleep(2)

#mc.send_angle(1,0,20)