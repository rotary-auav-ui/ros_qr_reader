import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import cv2
# Import the pyzbar library
from pyzbar.pyzbar import decode

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.get_logger().info("QR Code Detection Service Started (using pyzbar)")

        # --- Member Variables ---
        self.bridge = CvBridge()
        self.is_detecting = True
        # self.current_obj = 0

        # --- Subscribers & Publishers ---
        self.image_subscriber_realsense = self.create_subscription(Image, '/image_raw', self.image_callback, 10)  # subscribe rusb_cam
        self.image_subscriber_usb = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)  # subscribe realsense_ros
        self.publisher_content = self.create_publisher(String, 'qr_content', 10)
        self.publisher_center = self.create_publisher(Point, 'qr_center', 10)

    def image_callback(self, msg: Image):
        "Callback function for receiving and processing images."
        if not self.is_detecting:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Use pyzbar.decode to find and decode all QR codes in the frame
            results = decode(frame)

            # Process each detected QR code
            for result in results:
                # result.data is in bytes, so we must decode it to a string
                decoded_text = result.data.decode('utf-8')
                
                # result.polygon gives a list of points for the corners
                corners = np.array([p for p in result.polygon], dtype=np.int32)

                # Draw a box around every detected QR code
                cv2.polylines(frame, [corners], isClosed=True, color=(255, 128, 0), thickness=2)

                center_x = float(np.mean([p.x for p in result.polygon]))
                center_y = float(np.mean([p.y for p in result.polygon]))
                center_point = Point(x=center_x, y=center_y, z=0.0)

                # Publish direction and center point
                dir_msg = String()
                dir_msg.data = decoded_text
                self.publisher_content.publish(dir_msg)
                self.publisher_center.publish(center_point)

                # Highlight the valid, processed QR code
                cv2.putText(frame, decoded_text, (corners[0,0], corners[0,1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=3)

            # Display the processed frame
            cv2.imshow("QR Code Scanner (pyzbar)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.is_detecting = False
                self.get_logger().info("Detection stopped by user.")

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    qr_drone_service = QRCodeDroneService()
    rclpy.spin(qr_drone_service)
    qr_drone_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()