from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('COM5',115200)
mc.send_angle(1, 90, 20)
time.sleep(3)
mc.send_angle(5, -90, 20)
time.sleep(3)
mc.send_angle(2, 80, 20)
time.sleep(3)
# mc.send_angles([0, 0, 0, 0, 0, 0], 20)