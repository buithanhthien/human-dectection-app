import rclpy
import cv2
from robot.ros_bridge import RobotController
from perception.camera import Camera
from perception.detector import HumanDetector
from perception.tracker import HumanTracker
from control.controller import MotionController

rclpy.init()

camera = Camera()
detector = HumanDetector()
tracker = HumanTracker(frame_width=640)
controller = MotionController()
robot = RobotController()

try:
    while rclpy.ok():
        ret, frame = camera.read()
        if not ret:
            break
            
        results = detector.detect(frame)
        zone = tracker.get_human_position(results)
        
        linear, angular = controller.compute_velocity(zone)
        robot.move(linear, angular)
        
        rclpy.spin_once(robot, timeout_sec=0.01)
        
except KeyboardInterrupt:
    pass
finally:
    robot.stop()
    camera.release()
    robot.destroy_node()
    rclpy.shutdown()
