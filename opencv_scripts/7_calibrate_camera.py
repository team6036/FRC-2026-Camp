"""
Calibrates a camera given images with a chessboard with BOARD_SIZE inner corners and SQ_SIZE corner length (in meters).
Returns:
    - rms: root mean square (the error of calibration). A lower number is better.
    - K: camera matrix, which holds information about the lens using pixels as units. Important for PnP.
    - dist: distortion coefficients of the lens. Also important for PnP.
    - tvecs and rvecs: information about where the chessboard is every frame.
"""

import cv2
import numpy as np
import glob

BOARD_SIZE = (9, 7)
SQ_SIZE = 0.025

objp = np.array(
    [[x * SQ_SIZE, y * SQ_SIZE, 0]
     for y in range(BOARD_SIZE[1])
     for x in range(BOARD_SIZE[0])],
    dtype=np.float32
)

CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

obj_pts, img_pts = [], []
used_imgs = []
image_size = None

for path in sorted(glob.glob("images/*")):
    image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(image, BOARD_SIZE)
    if not ok: 
        print("FAIL", path)
        continue
    
    corners = cv2.cornerSubPix(image, corners, (11, 11), (-1, -1), CRITERIA)
    vis = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    obj_pts.append(objp)
    img_pts.append(corners)
    used_imgs.append(path)
    
    image_size = image.shape[::-1]  # (h, w) -> (w, h)

if len(obj_pts) < 10:
    print("Not enough valid images")
    exit()

rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, image_size, None, None)
print("RMS error:", rms)
print("Camera matrix:\n", K)
print("Distortion coefficients:\n", dist)