from pymycobot.mycobot import MyCobot


mc = MyCobot('COM6',115200)
mc.send_angles([50,6,45,34,-88,-38],20)

mc.send_angles([50,6,45,34,-88,-38],20)

mc.set_gripper_state(0,20,1)
