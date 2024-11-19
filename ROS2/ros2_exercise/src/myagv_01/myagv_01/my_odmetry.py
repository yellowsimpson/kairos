import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import time
import serial
import threading

class JDamr(object):
    def __init__(self, com="/dev/ttyUSB0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
       
        self.CMD_GET_ENCODER = 0x03

        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder3 = 0
        self.encoder4 = 0

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

    # Starting receiving thread 
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

    # Python thread for receiving packet 
    def receive_thread(self):
        try:
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    # parsing command from incoming packet 
    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            print(payload)
            encode1_str = payload[1:5]
            encode2_str = payload[5:9]
            encode3_str = payload[9:13]
            encode4_str = payload[13:17]
            self.encode1 = int.from_bytes(encode1_str, byteorder="big")
            print(self.encode1)
            self.encode2 = int.from_bytes(encode2_str, byteorder="big")
            print(self.encode2)
            self.encode3 = int.from_bytes(encode3_str, byteorder="big")
            print(self.encode3)
            self.encode4 = int.from_bytes(encode4_str, byteorder="big")
            print(self.encode4)

class MinimalPublicher(Node): 
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Int64, 'topic_encoder', 10)
        self.uga = JDamr('/dev/ttyUSB0')
        timer_period=0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Publish every 100ms
        self.uga.receive_thread()

    def timer_callback(self):
        msg = Int64()
        msg.data = self.uga.encode1  # Publish encode1 value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing encoder1: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    jdamr_driver_node = MinimalPublicher()

    rclpy.spin(jdamr_driver_node)

    jdamr_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
