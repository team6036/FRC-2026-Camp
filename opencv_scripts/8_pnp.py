"""
Given a camera calibration and real-life size of an AprilTag (in meters),
render the front-back, left-right, and up-down distances (in meters) of the tag relative to the camera,
as well as the rotation angle.
"""

import cv2
import numpy as np

TAG_SIZE = 0.1651
half = TAG_SIZE / 2
obj_pts = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0],
], dtype=np.float32)

detector = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
)

cap = cv2.VideoCapture(0)

K = np.array(
    [[1432.25451, 0.0, 936.983008],
     [0.0, 1430.55473, 526.171227],
     [0.0, 0.0, 1.0]]
)

dist_coeffs = np.array(
    [[0.00346364, 0.22647942, -0.00234237, -0.0023508, -0.43473895]]
)

while cap.isOpened():
    ok, frame = cap.read()
    if not ok:
        break

    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if ids is not None:
        for tag_corners, tag_id in zip(corners, ids):
            corner_pts = tag_corners[0].astype(int)
            cv2.polylines(frame, [corner_pts], True, (0, 255, 0), 5)

            _, rvec, tvec = cv2.solvePnP(obj_pts, tag_corners[0], K, dist_coeffs)

            lateral = tvec[0].item()  # l/r in meters
            vertical = tvec[1].item()  # u/d in meters
            depth = tvec[2].item()  # f/b in meters
            angle = np.degrees(np.linalg.norm(rvec))  # rotation angle in degrees
            text_lines = [
                f"ID: {tag_id[0]}",
                f"{np.linalg.norm(tvec):.2f} m away",
                f"{lateral:.2f} m right",
                f"{vertical:.2f} m above",
                f"{depth:.2f} m deep",
                f"Angled at {angle:.2f}deg"
            ]

            y_center = tag_corners[0].mean(axis=0).astype(int)[1]
            x_center = tag_corners[0].mean(axis=0).astype(int)[0]
            
            for i, line in enumerate(text_lines):
                text_y = y_center + i * 50
                
                # Get text size to draw background rectangle
                (text_width, text_height), baseline = cv2.getTextSize(
                    line,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    5
                )
                
                # Draw semi-transparent dark background
                overlay = frame.copy()
                cv2.rectangle(
                    overlay,
                    (x_center - 10, text_y - text_height - 10),
                    (x_center + text_width + 10, text_y + baseline + 10),
                    (0, 0, 0),
                    -1
                )
                frame = cv2.addWeighted(overlay, 0.7, frame, 0.3, 0)
                
                # Draw white text on top
                cv2.putText(
                    frame,
                    line,
                    (x_center, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (255, 255, 255),
                    5,
                )

        cv2.imwrite("demo.png", frame)
        cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()