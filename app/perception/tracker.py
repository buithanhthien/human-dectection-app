import supervision as sv
import numpy as np

class HumanTracker:
    def __init__(self, config):
        self.config = config
        self.tracker = sv.ByteTrack(
            track_activation_threshold=0.5,
            lost_track_buffer=30,
            minimum_matching_threshold=0.8,
            frame_rate=30
        )
        self.target_id = None
        self.lost_frames = 0
        self.max_lost_frames = 30
        
    def select_target(self, results):
        if not results or len(results[0].boxes) == 0:
            self.lost_frames += 1
            if self.lost_frames > self.max_lost_frames:
                self.target_id = None
            return None
        
        detections = sv.Detections.from_ultralytics(results[0])
        detections = self.tracker.update_with_detections(detections)
        
        valid_tracks = []
        for i, (bbox, conf, cls_id, track_id) in enumerate(zip(
            detections.xyxy, 
            detections.confidence, 
            detections.class_id,
            detections.tracker_id
        )):
            if conf < self.config.CONFIDENCE_THRESHOLD:
                continue
            
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            if area < self.config.MIN_BBOX_AREA:
                continue
                
            valid_tracks.append((bbox, area, track_id))
        
        if not valid_tracks:
            self.lost_frames += 1
            if self.lost_frames > self.max_lost_frames:
                self.target_id = None
            return None
        
        if self.target_id is not None:
            for bbox, area, track_id in valid_tracks:
                if track_id == self.target_id:
                    self.lost_frames = 0
                    return (bbox, track_id)
        
        largest = max(valid_tracks, key=lambda x: x[1])
        self.target_id = largest[2]
        self.lost_frames = 0
        return (largest[0], largest[2])
    
    def check_vertical_violation(self, bbox):
        y_min, y_max = bbox[1], bbox[3]
        return (y_min <= self.config.VERTICAL_MARGIN or 
                y_max >= self.config.FRAME_HEIGHT - self.config.VERTICAL_MARGIN)
    
    def get_bbox_center(self, bbox):
        return ((bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2)
    
    def get_bbox_height(self, bbox):
        return bbox[3] - bbox[1]
