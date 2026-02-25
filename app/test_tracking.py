import cv2
import supervision as sv
from perception.detector import HumanDetector
from perception.tracker import HumanTracker
from utils.config import Config

def main():
    config = Config()
    detector = HumanDetector()
    tracker = HumanTracker(config)
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    print("Press 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        results = detector.detect(frame)
        target = tracker.select_target(results)
        
        if target is not None:
            bbox, track_id = target
            x1, y1, x2, y2 = map(int, bbox)
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"ID: {track_id}", (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cx, cy = tracker.get_bbox_center(bbox)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        
        cv2.imshow("ByteTrack Test - ID Switching Solution", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
