import math

class GeometricRotationModel:
    def __init__(self, config):
        self.config = config
        
    def estimate_distance(self, bbox_height):
        if bbox_height == 0:
            return float('inf')
        return (self.config.REAL_HUMAN_HEIGHT * self.config.focal_length_px) / bbox_height
    
    def compute_rotation_angle(self, delta_px, distance):
        m = (delta_px * distance) / self.config.focal_length_px
        
        ratio = m / distance
        if abs(ratio) > 1.0:
            ratio = max(-1.0, min(1.0, ratio))
        
        theta = math.asin(ratio)
        
        if abs(theta) < self.config.THETA_DEADZONE:
            theta = 0
            
        return theta
    
    def compute_velocities(self, theta, distance):
        angular_z = -self.config.K_ANG * theta
        angular_z = max(self.config.MIN_ANGULAR_VELOCITY, min(angular_z, self.config.MAX_ANGULAR_VELOCITY))
        
        if abs(theta) < self.config.ANGLE_THRESHOLD and distance > self.config.DESIRED_DISTANCE:
            linear_x = self.config.K_LIN * (distance - self.config.DESIRED_DISTANCE)
            linear_x = max(self.config.MIN_LINEAR_VELOCITY, min(linear_x, self.config.MAX_LINEAR_VELOCITY))
        else:
            linear_x = 0.0
            
        return linear_x, angular_z
