"""
Uses color thresholding by filtering every pixel within two HSV bounds to detect yellow pixels.
Publishes results (center x, center y, target found) to NetworkTables for the robot to read,
and renders a bounding box on the frame.
"""


import cv2
from ntcore import NetworkTableInstance  # pip install pyntcore

inst = NetworkTableInstance.getDefault()
inst.startClient4("vision-client")
inst.setServerTeam(6036)
table = inst.getTable("Vision")

has_target_pub = table.getBooleanTopic("hasTarget").publish()
target_x_pub = table.getDoubleTopic("targetX").publish()
target_y_pub = table.getDoubleTopic("targetY").publish()

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        break

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = (20, 100, 100)
    upper = (35, 255, 255)
    mask = cv2.inRange(frame_hsv, lower, upper)
    # cv2.imshow("Mask", mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            # Get coordinates for a bounding box
            x, y, w, h = cv2.boundingRect(largest)
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            cv2.circle(frame, (center_x, center_y), 10, (0, 0, 255), -1)

            has_target_pub.set(True)
            target_x_pub.set(center_x)
            target_y_pub.set(center_y)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        else:
            has_target_pub.set(False)
    else:
        has_target_pub.set(False)

    cv2.imshow("Color Thresholding", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
inst.stopClient()