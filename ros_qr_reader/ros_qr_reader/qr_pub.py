import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Point
from vision_direction.srv import VisionDirection
import numpy as np
import cv2
import re
# Import the pyzbar library
from pyzbar.pyzbar import decode

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.get_logger().info("QR Code Detection Service Started (using pyzbar)")

        # --- Member Variables ---
        self.bridge = CvBridge()
        self.is_detecting = False
        self.current_obj = 0

        # --- Service ---
        self.srv = self.create_service(VisionDirection, 'start_stop_qr_detection', self.start_stop_callback)

        # --- Subscribers & Publishers ---
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher_direction = self.create_publisher(String, 'qr_direction', 10)
        self.publisher_target = self.create_publisher(Int8, 'qr_target', 10)
        self.publisher_center = self.create_publisher(Point, 'qr_center_bottom', 10)

    def start_stop_callback(self, request, response):
        "Service callback to start or stop QR code detection."
        if not 1 <= request.data <= 4:
            response.success = False
            response.message = f"Mission {request.data} error, please check your input!"
            return response

        self.current_obj = request.data
        self.is_detecting = True
        response.success = True
        response.message = f"QR Code detection started for mission {self.current_obj}."
        self.get_logger().info(response.message)
        return response

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

                # --- Validate the text format ---
                if not re.match(r'^\s*(?:[NSEW]\s*,\s*)*[NSEW]\s*,\s*\d+\s*$', decoded_text):
                    self.get_logger().warn(f"Ignoring QR with invalid format: '{decoded_text}'")
                    continue

                # --- If format is valid, process the data ---
                try:
                    sequence = [s.strip() for s in decoded_text.split(',')]
                    qr_target = int(sequence[-1])
                    direction = sequence[self.current_obj - 1]

                    # Calculate center point from the polygon corners
                    center_x = float(np.mean([p.x for p in result.polygon]))
                    center_y = float(np.mean([p.y for p in result.polygon]))
                    center_point = Point(x=center_x, y=center_y, z=0.0)

                    # Publish direction and center point
                    dir_msg = String()
                    dir_msg.data = direction
                    self.publisher_direction.publish(dir_msg)
                    self.publisher_center.publish(center_point)

                    self.get_logger().info(f"[Mission {self.current_obj}] Found valid QR. Direction: {direction}")

                    # Highlight the valid, processed QR code
                    cv2.putText(frame, decoded_text, (corners[0,0], corners[0,1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=3)

                    # --- Check if this is our mission target ---
                    if qr_target == self.current_obj:
                        self.get_logger().info(f"TARGET {self.current_obj} ACHIEVED!")
                        target_msg = Int8()
                        target_msg.data = qr_target
                        self.publisher_target.publish(target_msg)
                        self.is_detecting = False # Stop detection
                        break # Exit the loop since we found our target

                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"Error parsing valid QR data '{decoded_text}': {e}")

            # Display the frame
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