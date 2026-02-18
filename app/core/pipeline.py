class DetectionPipeline:
    def __init__(self, camera, detector):
        self.camera = camera
        self.detector = detector

    def run(self):
        while True:
            ret, frame = self.camera.read()
            if not ret:
                break

            results = self.detector.detect(frame)
            yield frame, results
