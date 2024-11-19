import sys
from cv_msg.srv import SrvArduino
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SrvArduino, 'motor_con')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SrvArduino.Request()

    def send_request(self, direction, speed):
        self.req.dir = direction
        self.req.speed = speed
        return self.cli.call_async(self.req)

def main():
    rclpy.init()

    if len(sys.argv) != 3:
        print("Usage: ros2 run ros_ardu ardu_cli <direction (f/r)> <speed (0-255)>")
        return

    direction = sys.argv[1]
    speed = int(sys.argv[2])

    if direction not in ['f', 'r']:
        print("Invalid direction! Use 'f' for forward and 'r' for reverse.")
        return

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(direction, speed)
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        f'Motor control: direction {direction}, speed {speed}, response: {response.answer}')

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
