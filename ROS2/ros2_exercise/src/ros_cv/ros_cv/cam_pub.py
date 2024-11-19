import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv_msg.msg import MsgCenter

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.center_publisher_ = self.create_publisher(MsgCenter, 'circle_center', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # BGR 이미지에서 HSV 이미지로 변환
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 파란색 범위 설정 (HSV 색상 공간)
            lower_green = np.array([35, 100, 100])
            upper_green = np.array([85, 255, 255])

            # 파란색 범위에 해당하는 마스크 생성
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # 마스크의 윤곽선 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # 가장 큰 윤곽선 찾기
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)

                if M['m00'] != 0:
                    # 중심점 계산
                    x = int(M['m10'] / M['m00'])
                    y = int(M['m01'] / M['m00'])
                    
                    # 반지름을 고정하거나 윤곽선의 외접 원을 계산할 수 있음
                    r = 30  # 고정된 반지름

                    # 원 그리기
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                    # MsgCenter 메시지에 좌표 설정
                    center_msg = MsgCenter()
                    center_msg.x = x
                    center_msg.y = y

                    # 메시지 퍼블리시
                    self.center_publisher_.publish(center_msg)
                    self.get_logger().info(f'Publishing: x={x}, y={y}')

            # 프레임 퍼블리시
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
