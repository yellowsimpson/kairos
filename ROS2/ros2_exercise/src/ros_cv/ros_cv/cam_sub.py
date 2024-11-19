import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_msg.msg import MsgCenter
import cv2

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # MsgCenter 메시지를 'circle_center' 토픽에서 구독
        self.center_subscription = self.create_subscription(
            MsgCenter,
            'circle_center',
            self.center_callback,
            10)
        self.center_subscription  # prevent unused variable warning

        # Image 메시지를 'video_frames' 토픽에서 구독
        self.image_subscription = self.create_subscription(
            Image,
            'video_frames',
            self.image_callback,
            10)
        self.image_subscription  # prevent unused variable warning

        self.br = CvBridge()
        self.last_center = None

    def center_callback(self, msg):
        # 수신된 메시지에서 x, y 좌표를 저장
        self.last_center = (msg.x, msg.y)
        self.get_logger().info(f'Received circle center: x={msg.x}, y={msg.y}')

    def image_callback(self, data):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        current_frame = self.br.imgmsg_to_cv2(data)

        # 마지막으로 수신된 중심 좌표에 원 그리기
        if self.last_center is not None:
            cv2.circle(current_frame, self.last_center, 30, (0, 255, 0), 4)
            cv2.rectangle(current_frame, (self.last_center[0] - 5, self.last_center[1] - 5),
                          (self.last_center[0] + 5, self.last_center[1] + 5), (0, 128, 255), -1)

        # OpenCV 창에 현재 프레임을 표시
        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # 노드를 명시적으로 종료
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

    # OpenCV 창 닫기
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()