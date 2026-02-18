import cv2
from perception.camera import Camera
from perception.detector import HumanDetector
from perception.visualizer import draw
from core.pipeline import DetectionPipeline

camera = Camera()
detector = HumanDetector()
pipeline = DetectionPipeline(camera, detector)

for frame, results in pipeline.run():
    annotated = draw(results)
    cv2.imshow("Human Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()
