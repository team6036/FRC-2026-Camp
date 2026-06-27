"""
An extension of the OpenCV demo script. Used time.time() to capture time delta between frames,
and calculate FPS (frames per second) of the camera feed, rendering it onto the video.
"""


import cv2
import time

cap = cv2.VideoCapture(0)

prev_time = time.time()

while True:
    ok, frame = cap.read()
    if not ok:
        break

    current_time = time.time()
    time_difference = current_time - prev_time

    fps = 1 / time_difference
    prev_time = current_time

    cv2.putText(frame, f"FPS: {fps:.2f}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2)

    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()