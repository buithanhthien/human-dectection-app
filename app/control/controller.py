class MotionController:
    def __init__(self, config, motion_model):
        self.config = config
        self.motion_model = motion_model
        
    def compute_velocity(self, bbox, tracker):
        if bbox is None:
            return 0.0, 0.0
        
        if tracker.check_vertical_violation(bbox):
            return 0.0, 0.0
        
        bbox_center_x, _ = tracker.get_bbox_center(bbox)
        bbox_height = tracker.get_bbox_height(bbox)
        
        delta_px = bbox_center_x - self.config.center_x
        distance = self.motion_model.estimate_distance(bbox_height)
        
        theta = self.motion_model.compute_rotation_angle(delta_px, distance)
        linear_x, angular_z = self.motion_model.compute_velocities(theta, distance)
        
        return linear_x, angular_z
