import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket
import pickle
import struct
import cv2
from cv_bridge import CvBridge
import threading

class CameraReceiver(Node):
    def __init__(self):
        super().__init__('camera_receiver')
        self.declare_parameter('port', 5005)
        self.declare_parameter('ip', '0.0.0.0')

        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.ip = self.get_parameter('ip').get_parameter_value().string_value

        # UDP 소켓 생성 및 바인딩
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, self.port))

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"Listening for video data on {self.ip}:{self.port}")
        self.receive_thread = threading.Thread(target=self.receive_video)
        self.receive_thread.start()

    def receive_video(self):
        while True:
            try:
                # 영상 데이터를 수신
                data, _ = self.sock.recvfrom(65536)  # 데이터 수신
                packet_size = struct.calcsize("Q")
                msg_size = struct.unpack("Q", data[:packet_size])[0]

                # 데이터 크기를 기반으로 실제 영상 데이터 추출
                frame_data = pickle.loads(data[packet_size:packet_size+msg_size])
                frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

                if frame is not None:
                    # 영상 데이터를 ROS 이미지 메시지로 변환
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.publisher.publish(ros_image)
                    self.get_logger().info("Published a frame")

            except Exception as e:
                self.get_logger().error(f"Error receiving video: {e}")

def main(args=None):
    rclpy.init(args=args)
    camera_receiver = CameraReceiver()
    rclpy.spin(camera_receiver)

    camera_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
