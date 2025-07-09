import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8
# from geometry_msgs.msg import Point, Polygon
# from vision_direction.srv import VisionDirection
import numpy as np
from qreader import QReader
import cv2
from cv2 import imshow, waitKey, putText, polylines, circle, FONT_HERSHEY_SIMPLEX, LINE_AA
# import re
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D, Pose2D, Point2D
# from geometry_msgs.msg import Pose2D
import math

class QRCodeDroneService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.bridge = CvBridge()
        self.QR = QReader(model_size="n")
        self.is_detecting = True
        self.current_obj = 0
        self.direction = ''
        self.front_frame: Image = None
        self.bot_frame: Image = None
        
        # Publishers
        self.qr_bbox_pub = self.create_publisher(BoundingBox2DArray, '/qr/bbox', 10)
        
        # Subscribers
        self.front_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.front_cam_callback, 10)
        self.bot_cam_sub = self.create_subscription(Image, '/image_raw', self.bot_cam_callback, 10)
        # self.image_subscriber = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback,10)
        # self.publisher_direction = self.create_publisher(String, 'qr_direction', 10)
        # self.publisher_content = self.create_publisher(String, 'qr_content', 10)
        # self.publisher_target = self.create_publisher(Int8, 'qr_target', 10)
        # self.publisher_center = self.create_publisher(Point, 'qr_center', 10)
        
        timer_period: float = 0.1
        self.timer = self.create_timer(timer_period, self.detect)
        self.success = True
        # self.detect()
    
    def front_cam_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.front_frame = frame  # Fixed: was setting bot_frame instead of front_frame
    
    def bot_cam_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.bot_frame = frame
    
    def detect(self):
        # Check if we have a frame to process
        if self.bot_frame is None:
            self.get_logger().warn("No frame received yet")
            return
        
        try:
            # Fixed: Create new QReader instance or reuse self.QR
            detection_results = self.QR.detect(image=self.bot_frame)
            
            # Check if detection_results is valid
            if not detection_results:
                return
                
            bbox_array = BoundingBox2DArray()  # Fixed: renamed from 'bbox' to avoid confusion
            
            # Fixed: Proper iteration over detection results
            for detection in detection_results:
                if isinstance(detection, dict):
                    # If detection is a dictionary directly
                    detection_data = detection
                elif isinstance(detection, tuple) and len(detection) == 2:
                    # If detection is a tuple (key, value)
                    _, detection_data = detection
                else:
                    continue
                    
                current = BoundingBox2D()
                
                # Extract center and corners with error checking
                if 'cxcy' in detection_data and 'quad_xy' in detection_data:
                    center = detection_data['cxcy']
                    corners = detection_data["quad_xy"]
                    
                    # Calculate theta using the box rotation method
                    theta: float = self.calc_theta(corners, center)
                    position = Point2D()
                    position.x = float(center[0])
                    position.y =  float(center[1])

                    # Set the pose - Fixed: Pose2D constructor
                    current.center = Pose2D()
                    current.center.position = position
                    current.center.theta =  theta
                    
                    # Set size if available
                    if 'wh' in detection_data:
                        current.size_x, current.size_y = float(detection_data['wh'][0]), float(detection_data['wh'][1])
                    
                    bbox_array.boxes.append(current)  # Fixed: append to boxes array
            
            # Publish the results
            self.qr_bbox_pub.publish(bbox_array)
            
        except Exception as e:
            self.get_logger().error(f"Error in detect(): {str(e)}")
    
    def calc_theta(self, corners, center):
        """
        Calculate the rotation angle of the QR code box.
        Uses the first corner to determine rotation angle.
        """
        try:
            center_x, center_y = center
            corner_x, corner_y = corners[0]
            
            # Calculate vector from center to first corner
            dx = corner_x - center_x
            dy = corner_y - center_y
            
            # Calculate angle using atan2
            theta = math.atan2(dy, dx)
            
            return theta
        except (IndexError, TypeError) as e:
            self.get_logger().error(f"Error calculating theta: {str(e)}")
            return 0.0

def main(args=None):
    rclpy.init(args=args)
    qr_drone_service = QRCodeDroneService()
    rclpy.spin(qr_drone_service)
    qr_drone_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()