import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import cv2
from pyzbar.pyzbar import decode

class CombinedQRCodeViewer(Node):
    def __init__(self):
        super().__init__('combined_qr_code_viewer')
        self.bridge = CvBridge()

        self.front_frame = None
        self.bottom_frame = None

        # MODIFIED: Create separate publishers for each camera's QR data
        self.front_content_pub = self.create_publisher(String, 'qr_content/front', 10)
        self.bottom_content_pub = self.create_publisher(String, 'qr_content/bottom', 10)
        self.front_center_pub = self.create_publisher(Point, 'qr_center/front', 10)
        self.bottom_center_pub = self.create_publisher(Point, 'qr_center/bottom', 10)
        
        self.vision_pub = self.create_publisher(CompressedImage, '/vision_img/compressed', 10)

        # subscriber
        self.create_subscription(Image, '/camera/color/image_raw', self.front_cam_callback, 10)  # realsense
        self.create_subscription(Image, '/image_raw', self.bottom_cam_callback, 10)  # usb_cam

        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Combined QR Viewer with Pyzbar Started.")  # logger info

    def front_cam_callback(self, msg: Image):
        """ for realsense cam """
        self.front_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def bottom_cam_callback(self, msg: Image):
        """ for usb cam """
        self.bottom_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # MODIFIED: Function now accepts publisher objects as arguments
    def process_qr(self, frame, content_publisher, center_publisher):
        """
        Detects QR codes in a frame, draws on the frame, and publishes data.
        """
        results = decode(frame)
        for result in results:
            decoded_text = result.data.decode('utf-8')
            corners = np.array([p for p in result.polygon], dtype=np.int32)

            # create rectangle of qr
            cv2.polylines(frame, [corners], isClosed=True, color=(255, 128, 0), thickness=2)
            cv2.putText(frame, decoded_text, (corners[0][0], corners[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # measure the center of qr
            center_x = float(np.mean([p.x for p in result.polygon]))
            center_y = float(np.mean([p.y for p in result.polygon]))

            # MODIFIED: Publish to the specific publisher passed to the function
            content_publisher.publish(String(data=decoded_text))
            center_publisher.publish(Point(x=center_x, y=center_y, z=0.0))
            
        return frame

    def timer_callback(self):
        if self.front_frame is None or self.bottom_frame is None:
            return

        try:
            front_img = self.front_frame.copy()
            bottom_img = self.bottom_frame.copy()

            # MODIFIED: Pass the correct publishers for each camera processing call
            front_img = self.process_qr(front_img, self.front_content_pub, self.front_center_pub)
            bottom_img = self.process_qr(bottom_img, self.bottom_content_pub, self.bottom_center_pub)

            # Resize to same height
            h1, w1, _ = front_img.shape
            h2, w2, _ = bottom_img.shape

            if h1 != h2:
                # Simple resize logic to make heights match for stacking
                if h1 > h2:
                    new_w2 = int(w2 * h1 / h2)
                    bottom_img = cv2.resize(bottom_img, (new_w2, h1))
                else:
                    new_w1 = int(w1 * h2 / h1)
                    front_img = cv2.resize(front_img, (new_w1, h2))

            stitched = np.hstack((bottom_img, front_img))

            # Publish stitched image
            ret, buffer = cv2.imencode('.jpg', stitched)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = buffer.tobytes()

            self.vision_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Failed in timer_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CombinedQRCodeViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()