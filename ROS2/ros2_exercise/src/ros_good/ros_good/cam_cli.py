import sys
import rclpy
from rclpy.node import Node
from cv_msg.srv import SrvGood
from cv_bridge import CvBridge
import cv2

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SrvGood, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SrvGood.Request()
        self.bridge = CvBridge()

    def send_request(self, cmd):
        self.req.cmd = cmd
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(minimal_client, future)
    
    if future.result() is not None:
        response = future.result()
        image = minimal_client.bridge.imgmsg_to_cv2(response.image, desired_encoding="bgr8")
        # Save the image
        filename = f"captured_image_{sys.argv[1]}.jpg"
        cv2.imwrite(filename, image)
        minimal_client.get_logger().info(f'Image saved as {filename}')
    else:
        minimal_client.get_logger().error('Service call failed')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()