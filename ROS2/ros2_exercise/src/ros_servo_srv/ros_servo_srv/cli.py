import sys
from uga_msg.srv import SrvServo
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SrvServo, 'servo_con')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SrvServo.Request()

    def send_request(self, degree):
        self.req.degree = degree 
        return self.cli.call_async(self.req)

def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Servo angle:%d' %response.result)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
