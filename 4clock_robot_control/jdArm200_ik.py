import numpy as np
import math
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
from util import create_tf_matrix, point_to_rad, e_to_r_matrix
import serial
import time

def send_angle(servo_angles):
    servo_angle_1,servo_angle_2,servo_angle_3 = servo_angles[0],servo_angles[1],servo_angles[2]
    cmd = "a"+str(round(servo_angle_1,2))+'b'+str(round(servo_angle_2,2))+'c'+ str(round(servo_angle_3,2)) +'d'+str(180)+'e\n'
    print(cmd)
    seq.write(cmd.encode())

def send_coord(xyz):
    servo_angle_1,servo_angle_2,servo_angle_3 = ik(xyz)
    cmd = "a"+str(round(servo_angle_1,2))+'b'+str(round(servo_angle_2,2))+'c'+ str(round(servo_angle_3,2)) +'d'+str(180)+'e\n'
    print(cmd)
    seq.write(cmd.encode())

def send_init():
    cmd = "a"+str(90)+'b'+str(100)+'c'+ str(90) +'d'+str(180)+'e\n'
    print(cmd)
    seq.write(cmd.encode())

def ik(xyz):
    unit = 1000
    rad_90 = np.pi/2

    link_1 = 0.092 * unit
    link_2 = 0.135 * unit
    link_3 = 0.147 * unit
    link_4 = 0.087 * unit

    servo_offset_1 = 90 * np.pi/180
    servo_offset_2 = 10 * np.pi/180
    servo_offset_3 = 45 * np.pi/180

    x,y,z = xyz[0],xyz[1],xyz[2]

    z_ = z-link_1                                                     # 암 시작하는 축에서 계산하기 위해서 z_ 변수
    theta_1 = point_to_rad(x,y)                                       # 0도에서 다른 점까지 각도
    theta_1 = (theta_1 - np.pi/2) * 2 + servo_offset_1                # 기어비가 1:2 라 원하는 각도로 가려면 서보모터는 2배 돌아야함
    len_A = norm([x,y]) - link_4
    len_A_ = norm([len_A,z_])

    alpha_1 = np.arccos(((link_2**2)+(link_3**2)-(len_A_**2))/(2*link_2*link_3))
    alpha_2 = np.arccos(((link_2**2)+(len_A_**2)-(link_3**2))/(2*link_2*len_A_))
    beta_1 = np.arctan2(z_,len_A)

    theta_2 = np.pi - (alpha_2 + beta_1) + servo_offset_2
    theta_3 = alpha_1 + alpha_2 + beta_1 - rad_90 + servo_offset_3

    servo_angle_1 = np.degrees(theta_1)
    servo_angle_2 = np.degrees(theta_2)
    servo_angle_3 = np.degrees(theta_3)
    # print(servo_angle_1,servo_angle_2,servo_angle_3)
    return [servo_angle_1,servo_angle_2,servo_angle_3]


seq = serial.Serial(
    port= 'COM13',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# send_init()
# time.sleep(3)

# send_angle([90,100,135,180])
# time.sleep(2)

send_coord([150,-150,100])
time.sleep(5)

send_coord([150,-150,200])
time.sleep(5)

send_coord([150,-150,100])
time.sleep(5)

send_coord([150,-150,200])
time.sleep(5)








