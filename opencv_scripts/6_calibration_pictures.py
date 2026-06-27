"""
Takes pictures of a calibration chessboard for camera calibration whenever SPACE is pressed.
Pictures are saved under the images/ folder.
"""

import cv2
import os

os.makedirs("images", exist_ok=True)
count = 0

cap = cv2.VideoCapture(0)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    cv2.imshow("Camera", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord(' '):
        filename = f"images/frame_{count:05d}.png"
        cv2.imwrite(filename, frame)
        print("Saved to", filename)
        count += 1

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()