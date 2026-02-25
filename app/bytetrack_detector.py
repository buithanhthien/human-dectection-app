from ultralytics import YOLO
import cv2
import numpy as np

class ByteTrackDetector:
    def __init__(self, model_path="yolov8n.pt", conf_threshold=0.5, iou_threshold=0.7):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        
    def detect_and_track(self, source=0):
        results = self.model.track(
            source=source,
            classes=[0],
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            tracker="bytetrack.yaml",
            stream=True,
            show=True
        )
        
        for result in results:
            if result.boxes.id is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                track_ids = result.boxes.id.cpu().numpy().astype(int)
                confidences = result.boxes.conf.cpu().numpy()
                
                for box, track_id, conf in zip(boxes, track_ids, confidences):
                    x1, y1, x2, y2 = box
                    print(f"ID: {track_id}, Conf: {conf:.2f}, Box: [{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]")
                    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ByteTrackDetector()
    detector.detect_and_track(source=0)
