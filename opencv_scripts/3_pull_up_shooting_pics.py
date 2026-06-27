"""
Part (1) of pull-up shooting, where the robot shoots if the human is detected to be shooting a ball.
Takes pictures for a classification model with two classes while the keys a and b are pressed.
Images taken while key a is pressed are saved under a directory a/ which can be batch labeled in Roboflow.
Images taken while key b is pressed are saved under a directory b/ which can be batch labeled in Roboflow.
"""


import cv2
import os

cap = cv2.VideoCapture(0)

os.makedirs("a", exist_ok=True)
os.makedirs("b", exist_ok=True)

count_a = 0
count_b = 0

while True:
    ok, frame = cap.read()
    if not ok:
        break

    cv2.imshow("Camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('a'):
        filename = f"a/frame_{count_a:05d}.png"
        cv2.imwrite(filename, frame)
        print("Saved to", filename)
        count_a += 1

    elif key == ord('b'):
        filename = f"b/frame_{count_b:05d}.png"
        cv2.imwrite(filename, frame)
        print("Saved to", filename)
        count_b += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()