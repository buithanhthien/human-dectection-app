import os
import math
from dotenv import load_dotenv

load_dotenv()

class Config:
    FRAME_WIDTH = int(os.getenv('FRAME_WIDTH', 640))
    FRAME_HEIGHT = int(os.getenv('FRAME_HEIGHT', 480))
    HORIZONTAL_FOV = float(os.getenv('HORIZONTAL_FOV', 1.0472))
    
    CONFIDENCE_THRESHOLD = float(os.getenv('CONFIDENCE_THRESHOLD', 0.5))
    MIN_BBOX_AREA = int(os.getenv('MIN_BBOX_AREA', 1000))
    
    K_ANG = float(os.getenv('K_ANG', 1.0))
    K_LIN = float(os.getenv('K_LIN', 0.5))
    
    MAX_ANGULAR_VELOCITY = float(os.getenv('MAX_ANGULAR_VELOCITY', 1.5))
    MIN_ANGULAR_VELOCITY = float(os.getenv('MIN_ANGULAR_VELOCITY', -1.5))
    MAX_LINEAR_VELOCITY = float(os.getenv('MAX_LINEAR_VELOCITY', 0.6))
    MIN_LINEAR_VELOCITY = float(os.getenv('MIN_LINEAR_VELOCITY', 0.0))
    
    THETA_DEADZONE = float(os.getenv('THETA_DEADZONE', 0.02))
    ANGLE_THRESHOLD = float(os.getenv('ANGLE_THRESHOLD', 0.1))
    
    VERTICAL_MARGIN = int(os.getenv('VERTICAL_MARGIN', 10))
    
    REAL_HUMAN_HEIGHT = float(os.getenv('REAL_HUMAN_HEIGHT', 1.7))
    DESIRED_DISTANCE = float(os.getenv('DESIRED_DISTANCE', 1.5))
    
    @property
    def center_x(self):
        return self.FRAME_WIDTH / 2
    
    @property
    def center_y(self):
        return self.FRAME_HEIGHT / 2
    
    @property
    def focal_length_px(self):
        return self.FRAME_WIDTH / (2 * math.tan(self.HORIZONTAL_FOV / 2))
