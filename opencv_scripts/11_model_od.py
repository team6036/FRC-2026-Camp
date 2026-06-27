"""
Runs inference on an object detection model. The model should be under runs/detect/train/weights/best.pt.
Confidence of the model can be adjusted with the conf= parameter in model.predict().
"""

import cv2
import ultralytics

model = ultralytics.YOLO("runs/detect/train/weights/best.pt")

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        break

    results = model.predict(frame, conf=0.25)
    annotated = results[0].plot()
    cv2.imshow("Object Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()