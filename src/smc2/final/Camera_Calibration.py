import cv2
import numpy as np
import cv2.aruco as aruco
import time

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_length = 0.008   # meters
square_length = 0.012   # meters

board = aruco.CharucoBoard(
    size=(10, 7),
    squareLength=square_length,
    markerLength=marker_length,
    dictionary=aruco_dict
)

url = "http://klasthorgren:video123@10.37.196.204:8081/video"
url = "http://10.37.125.170:8080/video"
cap = cv2.VideoCapture(url)
if not cap.isOpened():
    print("Cant open camera, exiting")
    exit()
all_corners = []
all_ids = []
img_size = None

print("Move the board slowly. Press 'q' to finish.")

last_time = 0.0
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    now = time.time()

    if now - last_time > 0.5:
        last_time = now
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

        if ids is not None:
            _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=board
            )
        
            if charuco_ids is not None:
                if len(charuco_ids) > 35:
                    print("Corners detected this frame:", len(charuco_ids))
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    img_size = gray.shape[::-1]

    cv2.imshow("Calibration", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Calibrate
ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
    all_corners,
    all_ids,
    board,
    img_size,
    None,
    None
)

np.savez("camera_charuco_calibration.npz",
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs)

print("Calibration complete. Saved to camera_charuco_calibration.npz")
