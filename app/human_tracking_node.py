#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'app'))

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
    
    def control_loop(self):
        linear_x, angular_z = self.controller.compute_velocity(self.latest_bbox, self.tracker)
        
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

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
