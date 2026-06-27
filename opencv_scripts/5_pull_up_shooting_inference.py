"""
Part (3) of pull-up shooting. Requires the ultralytics and pyntcore packages, which can be installed with pip.
Requires the model saved under runs/classify/train/weights/best.pt.

Calls model() on each frame of the camera feed, and publishes a boolean to the NetworkTables server for the robot.
"""

import ultralytics
import cv2
import ntcore

model = ultralytics.YOLO("runs/classify/train/weights/best.pt")

cap = cv2.VideoCapture(0)

inst = ntcore.NetworkTableInstance.getDefault()
inst.startClient4("vision-client")
inst.setServerTeam(6036)

table = inst.getTable("Vision")
pub = table.getBooleanTopic("Shoot").publish()
pub.set(False)


while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        break

    results = model(frame, verbose=False)
    r = results[0]

    class_id = r.probs.top1
    confidence = r.probs.top1conf
    label = r.names[class_id]

    if label == "Shooting" and confidence > 0.8:
        pub.set(True)
    else:
        pub.set(False)

    text = f"{label} {confidence:.2f}"
    cv2.putText(frame, text, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 0, 0), 3)

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()