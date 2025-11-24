import cv2
import cv2.aruco as aruco
import numpy as np

# Load calibration file
data = np.load("camera_charuco_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

# Your iPhone stream URL
url = "http://klasthorgren:video123@10.15.177.67:8081/video"
cap = cv2.VideoCapture(url)

# ArUco dictionary (must match markers on Jenga blocks)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Physical marker size on Jenga blocks (meters)
marker_length = 0.0225  # 22.5 mm

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw detected markers
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            marker_length,
            camera_matrix,
            dist_coeffs
        )

        for rvec, tvec in zip(rvecs, tvecs):
            # Draw axis on the marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

            # Print 3D position of marker center relative to camera
            print("Marker tvec (x, y, z) meters:", tvec.flatten())

    cv2.imshow("Jenga Marker Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()