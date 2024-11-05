import serial 
import time  

ser = serial.Serial('COM6', 115200, timeout=1)
time.sleep(2)

def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command}") 

try:
    while True:
        command = input("Enter 'q' to stop servo, 'r' to move servo: ")
        if command == 'q' or command == 'r':  # 'q'는 girpper close, 'r'은 girpper open
            send_command(command)  # 아두이노로 명령 전송
        else:
            print("Invalid command. Please enter 'q' or 'r'.")
except KeyboardInterrupt:
    print("Program interrupted")

finally:
    ser.close()
