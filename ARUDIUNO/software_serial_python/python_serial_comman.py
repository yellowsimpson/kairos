import serial
import time 
import threading

# 시리얼 포트 설정
port = 'COM3'  # 사용하는 시리얼 포트 이름 변경 필요
baudrate = 9600  # 보드레이트 설정
timeout = 1  # 타임아웃 설정 (초 단위)

ser = serial.Serial(port, baudrate, timeout=timeout)
count = 0

def receive_serial():
    while True:
        if ser.inWaiting():
            command = ser.readline()
            cmd_dec = command.decode()
            print(cmd_dec)
            time.sleep(0.05)

task = threading.Thread(target=receive_serial)
task.start()    

while True:
    # 보낼 문자열 설정
    message = 'a'+str(count)+'b\n'
    count+=1

    # 문자열 전송
    print(message)
    ser.write(message.encode('utf-8'))

    # 시리얼 포트 닫기
    time.sleep(1)
