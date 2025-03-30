import serial 
import time 

# 시리얼 포트 설정
port = 'COM3'  # 사용하는 시리얼 포트 이름 변경 필요
baudrate = 9600  # 보드레이트 설정
timeout = 1  # 타임아웃 설정 (초 단위)

ser = serial.Serial(port, baudrate, timeout=timeout)
#포트 설정하는 코드
while True:
     if ser.inWaiting():
        command = ser.readline()
        cmd_dec = command.decode()
        print(command)

