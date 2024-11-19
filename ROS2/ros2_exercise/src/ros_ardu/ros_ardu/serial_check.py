import serial
import time

port ='/dev/ttyUSB0'
baudrate=115200
timeout=1

ser=serial.Serial(port,baudrate,timeout=timeout)
while True:
    ser.write('a'.encode())
    time.sleep(1)
    ser.write('b'.encode())
    time.sleep(1)