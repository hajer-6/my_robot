#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO

class LetterCollector:
    def __init__(self, model_path, num_letters=5, cap=None):
        self.model_path = model_path
        self.num_letters = num_letters
        self.detected_letters = []
        self.classes_map = {i: chr(65 + i) for i in range(26)}
        self.model = YOLO(self.model_path)

        # Use existing camera if passed
        self.cap = cap
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError("Camera did not open")

        self.output_file = "/home/hajer/catkin_ws/src/my_robot_package/computer vision/output_file/results.txt"

    @staticmethod
    def is_green_background(frame, bbox, expand=10, green_thresh=0.12):
        x1, y1, x2, y2 = bbox
        h, w = frame.shape[:2]
        x1, y1 = max(0, x1 - expand), max(0, y1 - expand)
        x2, y2 = min(w - 1, x2 + expand), min(h - 1, y2 + expand)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 60, 60])
        upper_green = np.array([95, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        green_ratio = np.count_nonzero(green_mask) / float(roi.shape[0] * roi.shape[1])
        return green_ratio > green_thresh

    def detect_letters(self, frame):
        results = self.model(frame, conf=0.25)
        detections = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = self.classes_map.get(cls, str(cls))
                detections.append({"label": label, "confidence": conf, "bbox": (x1, y1, x2, y2)})
        return detections

    def save_word(self):
        final_word = "".join(self.detected_letters)
        print(f"\nFinal Word: {final_word}")
        with open(self.output_file, "w") as f:
            f.write(final_word)
        print(f"📝 Saved to {self.output_file}")
