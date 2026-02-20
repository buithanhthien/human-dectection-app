#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import sys
import os

workspace_root = os.environ.get('HUMAN_DETECTION_APP_ROOT', '/home/thien/human_detection_app')
app_path = os.path.join(workspace_root, 'app')
if not os.path.exists(app_path):
    workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
    app_path = os.path.join(workspace_root, 'app')
sys.path.insert(0, app_path)

from utils.config import Config
from perception.detector import HumanDetector
from perception.tracker import HumanTracker
from control.motion_model import GeometricRotationModel
from control.controller import MotionController

class HumanTrackingNode(Node):
    def __init__(self):
        super().__init__('human_tracking_node')
        
        self.config = Config()
        self.bridge = CvBridge()
        
        self.detector = HumanDetector()
        self.tracker = HumanTracker(self.config)
        self.motion_model = GeometricRotationModel(self.config)
        self.controller = MotionController(self.config, self.motion_model)
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.latest_bbox = None
        
        self.get_logger().info('Human Tracking Node Started')
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.detector.detect(frame)
        self.latest_bbox = self.tracker.select_target(results)
        
        if self.latest_bbox is not None:
            self.get_logger().info(f'Human detected! BBox: [{self.latest_bbox[0]:.0f}, {self.latest_bbox[1]:.0f}, {self.latest_bbox[2]:.0f}, {self.latest_bbox[3]:.0f}]')
        else:
            self.get_logger().info('No human detected', throttle_duration_sec=2.0)
    
    def control_loop(self):
        linear_x, angular_z = self.controller.compute_velocity(self.latest_bbox, self.tracker)
        
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        
        if self.latest_bbox is not None:
            self.get_logger().info(f'Cmd: linear={linear_x:.2f}, angular={angular_z:.2f}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
