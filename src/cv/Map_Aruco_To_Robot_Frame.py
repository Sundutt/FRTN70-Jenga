import cv2
import cv2.aruco as aruco
import numpy as np

# ===============================
# 1. Load Camera Calibration
# ===============================
data = np.load("camera_charuco_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

# ===============================
# 2. Video stream from iPhone
# ===============================
url = "http://klasthorgren:video123@192.168.1.170:8081/video"
cap = cv2.VideoCapture(url)

# ===============================
# 3. ArUco dictionary & parameters
# ===============================
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Physical marker size (meters) of all ArUco markers
marker_length = 0.1434  # Example: 14.34 mm

# ================================================================
# 4. Provide: Transform of CALIBRATION MARKER relative to ROBOT BASE
# ================================================================
# YOU MUST MEASURE THIS ONCE using the robot.
# Position in meters:
base_marker_pos = np.array([0.155, -0.122, 0.000])  # example values

# Rotation of the calibration marker relative to robot base as 3×3 rotation matrix:
# Example: Marker lying flat on robot table with X forward, Y left, Z up.
base_marker_rot = np.array([[-1, 0, 0],
                            [0, -1, 0],
                            [0, 0, 1]])

# Construct T_base_marker (4×4 homogeneous transform)
T_base_marker = np.eye(4)
T_base_marker[:3, :3] = base_marker_rot
T_base_marker[:3, 3] = base_marker_pos

# We compute this ONCE from the first frame where the marker is detected
T_base_cam = None


# ===============================
# 5. Helper: rvec+tvec → 4×4 pose
# ===============================
def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


# ===============================
# 6. Main Loop
# ===============================
print("Running. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )

    if ids is not None:
        # Draw marker borders
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Pose estimation
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            marker_length,
            camera_matrix,
            dist_coeffs
        )

        for (marker_id, rvec, tvec) in zip(ids.flatten(), rvecs, tvecs):

            # --------------------------
            # 6A. Convert to T_cam_marker
            # --------------------------
            T_cam_marker = rvec_tvec_to_T(rvec, tvec)

            # --------------------------
            # 6B. Calibration: compute T_base_cam once
            # --------------------------
            if T_base_cam is None and marker_id == 0:  # Use marker ID 0 as calibration marker
                # For the calibration marker:
                # T_base_cam = T_base_marker * inv(T_cam_marker)
                T_base_cam = T_base_marker @ np.linalg.inv(T_cam_marker)
                print("\nComputed T_base_cam:\n", T_base_cam)

            # --------------------------
            # 6C. If camera extrinsics known → compute block pose
            # --------------------------
            if T_base_cam is not None:
                # T_base_block = T_base_cam * T_cam_block
                T_base_block = T_base_cam @ T_cam_marker

                block_pos = T_base_block[:3, 3]
                block_rot = T_base_block[:3, :3]

                print(f"\nMarker ID {marker_id}")
                print("Position in robot base frame (m):")
                print(block_pos)
                print("Rotation matrix in robot base frame:")
                print(block_rot)

                # Draw the coordinate axes for visualization
                cv2.drawFrameAxes(
                    frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03
                )

    cv2.imshow("Mapped ArUco Markers to Robot Base Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()