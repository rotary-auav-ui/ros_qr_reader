import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from vision_direction.srv import VisionDirection


class QrCodeClient(Node):
    def __init__(self):
        super().__init__('qr_code_client')
        self.cli = self.create_client(VisionDirection, 'start_stop_qr_detection') # Client for your service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # self.req = VisionDirection.Request()

    def send_request(self, mission_number):
        req = VisionDirection.Request()
        req.data = mission_number
        future = self.cli.call_async(req)
        return future

def main(args=None):
    rclpy.init()
    mission_number = int(input("Input mission (1-4): "))

    client = QrCodeClient()
    future = client.send_request(mission_number)
    rclpy.spin_until_future_complete(client, future)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
