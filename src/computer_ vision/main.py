from cv_signs import Sign_Node
from cv_letters import LetterCollector
import cv2
import rospy 
from std_msgs.msg import String

if __name__ == "__main__":
    sign_model = "/home/rawan/catkin_ws/src/Robot-Competition/models/SignModel.pt"
    alphabet_model = "/home/rawan/catkin_ws/src/Robot-Competition/models/alphabet.pt"

    # Main camera
    main_cap = cv2.VideoCapture(0)
    if not main_cap.isOpened():
        raise RuntimeError("Main camera could not be opened")

    # Detectors using the same camera
    sign_detector = Sign_Node(sign_model, cap=main_cap)
    alphabet_detector = LetterCollector(alphabet_model, cap=main_cap)

    detection_window_open = False
    detection_window_name = ""
    confidence_threshold = 0.5

    print("Robot walking ifyou want to finish  press 'q' to quit.")

    while True:
        ret, frame = main_cap.read()
        if not ret:
            break
        frame = cv2.flip(frame, 1)

        # ----------- Sign Detection (Priority) -----------
        sign_results = [d for d in sign_detector.detect_signs(frame) if d["confidence"] > confidence_threshold]
        if len(sign_results) > 0:
            processed_frame = sign_detector.draw_detections(frame.copy(), sign_results)
            if not detection_window_open or detection_window_name != "Sign Detector":
                if detection_window_open:
                    cv2.destroyWindow(detection_window_name)
                detection_window_name = "Sign Detector"
                cv2.namedWindow(detection_window_name, cv2.WINDOW_NORMAL)
                detection_window_open = True
            cv2.imshow(detection_window_name, processed_frame)
            cv2.waitKey(1)
            continue

        # ----------- Letter Detection -----------
        letter_results = [d for d in alphabet_detector.detect_letters(frame) if d["confidence"] > confidence_threshold]
        if len(letter_results) > 0:
            processed_frame = frame.copy()
            for r in letter_results:
                x1, y1, x2, y2 = r["bbox"]
                label = r["label"]
                conf = r["confidence"]
                color = (0, 255, 0)
                cv2.rectangle(processed_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(processed_frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            if not detection_window_open or detection_window_name != "Letter Detection":
                if detection_window_open:
                    cv2.destroyWindow(detection_window_name)
                detection_window_name = "Letter Detection"
                cv2.namedWindow(detection_window_name, cv2.WINDOW_NORMAL)
                detection_window_open = True
            cv2.imshow(detection_window_name, processed_frame)
            cv2.waitKey(1)
            continue

        # ----------- Nothing Detected -----------
        if detection_window_open:
            cv2.destroyWindow(detection_window_name)
            detection_window_open = False

        cv2.imshow("Main Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    main_cap.release()
    cv2.destroyAllWindows()
