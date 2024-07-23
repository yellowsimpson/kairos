import serial

ser = serial.Serial('COM3', 115200)

data = ['uga', 'uga1', 'uga2', 'uga3']
data_str = ','

for i in range(len(data)):
    data_str += data[i]
    data_str += ','

print(data_str)
ser.write(data_str.encode())

