class HumanTracker:
    def __init__(self, config):
        self.config = config
        
    def select_target(self, results):
        if not results or len(results[0].boxes) == 0:
            return None
        
        valid_boxes = []
        for box in results[0].boxes:
            if box.conf[0] < self.config.CONFIDENCE_THRESHOLD:
                continue
            
            xyxy = box.xyxy[0].cpu().numpy()
            area = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
            
            if area < self.config.MIN_BBOX_AREA:
                continue
                
            valid_boxes.append((box, area))
        
        if not valid_boxes:
            return None
            
        largest_box = max(valid_boxes, key=lambda x: x[1])[0]
        return largest_box.xyxy[0].cpu().numpy()
    
    def check_vertical_violation(self, bbox):
        y_min, y_max = bbox[1], bbox[3]
        return (y_min <= self.config.VERTICAL_MARGIN or 
                y_max >= self.config.FRAME_HEIGHT - self.config.VERTICAL_MARGIN)
    
    def get_bbox_center(self, bbox):
        return ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    
    def get_bbox_height(self, bbox):
        return bbox[3] - bbox[1]
