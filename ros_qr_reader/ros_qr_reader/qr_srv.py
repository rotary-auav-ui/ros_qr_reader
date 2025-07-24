import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from pyzbar.pyzbar import decode

from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from qr_interfaces.msg import Qrcontent
from qr_interfaces.srv import Qrsrv
from vision_msgs.msg import BoundingBox2DArray, BoundingBox2D, Pose2D, Point2D


class QrService(Node):
    def __init__(self):
        super().__init__('qr_code_drone_service')
        self.bridge = CvBridge()

        self.is_scanning = True
        self.front_frame: Image = None
        self.bot_frame: Image = None
        self.currentz_cam: str = None

        # Service Server
        # self.qr_server = self.create_service(Qrsrv, '/qr/start', 10)
        
        # Publishers
        self.qr_bbox_pub = self.create_publisher(BoundingBox2DArray, '/qr/bbox', 10)
        self.qr_current_cam_pub = self.create_publisher(String, '/qr/current_cam', 10)
        self.qr_is_scanning_pub = self.create_publisher(Bool, '/qr/is_scanning', 10)
        self.qr_content_pub = self.create_publisher(Qrcontent, '/qr/content', 10)
        
        # Subscribers
        self.front_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.front_cam_callback, 10)
        self.bot_cam_sub = self.create_subscription(Image, '/image_raw', self.bot_cam_callback, 10)
        

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
        # if self.is_scanning is False:
        #     return
        
        if self.bot_frame is None:
            self.get_logger().warn("No frame received yet")
            return
        
        bbox_array = BoundingBox2DArray()

        try:  
            detection_results = decode(self.bot_frame)
            
            # Check if detection_results is valid
            if not detection_results:
                return
            
            # publish current_cam
            current_cam_msg = String()
            current_cam_msg.data = "bottom"
            self.qr_current_cam_pub.publish(current_cam_msg)

            # publish qr
            qr_content_msg = Qrcontent()

            # publish BoundingBox    
            bbox_array = BoundingBox2DArray() 
            
            for detection in detection_results:                    
                current = BoundingBox2D()
                
                # treat top-left point as center
                position = Point2D()
                position.x = float(detection.rect.left)
                position.y =  float(detection.rect.top)

                current.center = Pose2D()
                current.center.position = position
                current.center.theta = 0.0 # no need for rotation

                current.size_x = float(detection.rect.width)
                current.size_y = float(detection.rect.height)

                current_decode = detection.data.decode('utf-8')

                bbox_array.boxes.append(current)
                qr_content_msg.data.append(current_decode)            
            
            self.qr_content_pub.publish(qr_content_msg)
            self.qr_bbox_pub.publish(bbox_array)
        
        except Exception as e:
            self.get_logger().error(f"Error in detect(): {str(e)}")
    
def main(args=None):
    rclpy.init(args=args)
    qr_service = QrService()
    rclpy.spin(qr_service)
    qr_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()