import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point

import re

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('get_qr_content')
        self.subscription = self.create_subscription(String,'qr_content',self.listener_callback,10)
        self.subscription_center = self.create_subscription(Point,'qr_center_',self.center_callback,10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f"QR content: \"{msg.data}\"")

        pattern = r'^\s*(?:[NSEW]\s*,\s*)*[NSEW]\s*,\s*\d+\s*$'
        if not re.match(pattern, msg.data):
            self.get_logger().warn("Received QR with invalid format.")
        else:
            self.get_logger().info("QR format is valid.")
                    

    def center_callback(self, msg):
        self.get_logger().info(f"center_x = {msg.x}, center_y = {msg.y}")


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()