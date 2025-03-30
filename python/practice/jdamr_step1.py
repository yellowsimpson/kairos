'''
/*MIT License

Copyright (c) 2024 JD edu. http://jdedu.kr author: conner.jeong@gmail.com
     
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
     
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
     
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN TH
SOFTWARE.*/

'''

import serial
import threading 
import struct
import time 

class JDamr(object):
    def __init__(self, com="COM4"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x01
        self.CMD_GET_SPEED = 0x02
        self.CMD_GET_ENCODER = 0x03

        self.CMD_CAR_RUN = 0x04

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")
        time.sleep(1)
    
    '''
    Protocol 
    - Packets have following bytes.
      - Header byte 
      - length byte
      - command byte
      - payload bytes 
      - checksum byte 
    '''
    def receive_data(self):     
        self.ser.flushInput()
        while True:
            head = bytearray(self.ser.read())[0]
            if head == self.HEAD:
                length = bytearray(self.ser.read())[0]  
                payload = [] 
                for i in range(length-1):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    def receive_thread(self):
        try:
            task_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=task_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            encode1_str = payload[1:5]
            print(encode1_str)
            encode1 =int.from_bytes(encode1_str, byteorder="big")
            print(encode1)

if __name__ == '__main__':
    com = 'COM4'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()
   
    while True:
       pass


