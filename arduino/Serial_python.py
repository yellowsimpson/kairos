import serial
#시리얼 모니터를 할때 다른 모니터는 다 꺼야함
import time
import random
seq = serial.Serial(
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
seq.port = "COM3"
a = 0
b = 0
c = 0
d = 0
seq.open()
cnt=0
while True:
    print(cnt)
    cnt+=1
    data = 'a'
    data += str(random.randint(0, 100))
    data += 'b'
    # data += str(random.randint(0, 100))
    # data += 'c'
    # data += str(random.randint(0, 100))
    # data += 'd'
    # data += str(random.randint(0, 100))
    # data += 'e'
    seq.write(data.encode())
    time.sleep(2.5)
    