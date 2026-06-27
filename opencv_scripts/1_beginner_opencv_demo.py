"""
Introductory demo for OpenCV. Requires a webcam or a plugged-in camera.
OpenCV can be installed with pip, Python's package manager: pip install opencv-python
"""


import cv2

cap = cv2.VideoCapture(0)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    cv2.imshow("Camera", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()