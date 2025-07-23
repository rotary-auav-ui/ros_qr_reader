import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

import cv2
import math
import numpy as np
from qreader import QReader

from sensor_msgs.msg import Image
from std_msgs.msg import String, Int8, Bool
from vision_direction.srv import VisionDirection
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D, Pose2D, Point2D

class QrService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.bridge = CvBridge()
        self.QR = QReader(model_size="n")

        self.is_scanning = True
        self.front_frame: Image = None
        self.bot_frame: Image = None
        # self.success = True

        # Service Server
        self.qr_server = self.create_service(VisionDirection, '/qr/start', 10)
        
        # Publishers
        self.qr_bbox_pub = self.create_publisher(BoundingBox2DArray, '/qr/bbox', 10)
        self.qr_current_cam_pub = self.create_publisher(String, '/qr/current_cam', 10)
        self.qr_is_scanning_pub = self.create_publisher(Bool, '/qr/is_scanning', 10)
        
        # Subscribers
        self.front_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.front_cam_callback, 10)
        self.bot_cam_sub = self.create_subscription(Image, '/camera/front', self.bot_cam_callback, 10)
        

        timer_period: float = 0.1
        self.detect_timer = self.create_timer(timer_period, self.detect)
        self.is_scanning_timer = self.create_timer(timer_period, self.is_scanning_callback)
    
    def front_cam_callback(self, msg: Image):
        self.front_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
    def bot_cam_callback(self, msg: Image):
        self.bot_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def is_scanning_callback(self):
        msg = Bool()
        msg.data = self.is_scanning
        self.qr_is_scanning_pub.publish(msg)
         
    
    def detect(self):
        if self.is_scanning is False:
            return
        
        if self.bot_frame is None:
            self.get_logger().warn("No frame received yet")
            return
        
        bbox_array = BoundingBox2DArray()

        try:
            detection_results = self.QR.detect(image=self.bot_frame)
            
            # Check if detection_results is valid
            if not detection_results:
                return
            
            # publish current_cam
            current_cam_msg = String()
            current_cam_msg.data = "bottom"
            self.qr_current_cam_pub.publish(current_cam_msg)

            # publish BoundingBox    
            bbox_array = BoundingBox2DArray() 
            
            for detection in detection_results:
                if isinstance(detection, dict):
                    # If detection is a dictionary directly
                    detection_data = detection
                elif isinstance(detection, tuple) and len(detection) == 2:
                    # If detection is a tuple (key, value)
                    key, detection_data = detection
                else:
                    continue
                    
                current = BoundingBox2D()
                
                if 'cxcy' in detection_data and 'quad_xy' in detection_data:
                    center = detection_data['cxcy']
                    corners = detection_data["quad_xy"]
                    
                    # Calculate theta using the box rotation method
                    # theta: float = self.calc_theta(corners, center)
                    position = Point2D()
                    position.x = float(center[0])
                    position.y =  float(center[1])

                    current.center = Pose2D()
                    current.center.position = position
                    # current.center.theta =  theta
                    current.center.theta = 0.0
                    
                    if 'wh' in detection_data:
                        current.size_x, current.size_y = float(detection_data['wh'][0]), float(detection_data['wh'][1])
                    
                    bbox_array.boxes.append(current)
            
        except Exception as e:
            self.get_logger().error(f"Error in detect(): {str(e)}")
            
        finally:
            # Publish the results
            self.qr_bbox_pub.publish(bbox_array)
    
    def calc_theta(self, corners, center) -> float:
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
    qr_service = QrService()
    # executor = MultiThreadedExecutor(num_threads=1)

    # executor.add_node(qr_service)
    
    # try:
    #     executor.spin()
    # finally:
    #     qr_service.destroy_node()
    #     rclpy.shutdown()

    rclpy.spin(qr_service)
    qr_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()