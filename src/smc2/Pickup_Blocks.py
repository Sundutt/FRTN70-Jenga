import cv2
import cv2.aruco as aruco
import numpy as np
import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R_scipy
import time

ROBOT_IP = "192.168.1.150"

rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

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

# Marker sizes (meters)
marker_size_calibration = 0.030   # ID 0 (bigger marker)
marker_size_block = 0.015         # ID 1 and upwards (smaller markers)

# ================================================================
# 4. Provide: Transform of CALIBRATION MARKER relative to ROBOT BASE
# ================================================================
# YOU MEASURE THIS ONCE
base_marker_pos = np.array([0.350, -0.120, 0.000])  # example
base_marker_rot = np.eye(3)

T_base_marker = np.eye(4)
T_base_marker[:3, :3] = base_marker_rot
T_base_marker[:3, 3] = base_marker_pos

# Will be computed from marker ID 0
T_base_cam = None

# Keeps track of which block (ID) to pick next
next_block_id = 1


# ===============================
# Helper: rvec+tvec → 4×4 transform
# ===============================
def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

def moveL(t, R, speed=0.2, accel=0.4):
    """
    Move UR robot linearly (moveL) using a rotation matrix and translation vector.

    Parameters:
        rtde_c : RTDEControlInterface instance
        R      : 3x3 rotation matrix (numpy array)
        t      : 3x1 translation vector [x, y, z] in meters
        speed  : linear speed (m/s)
        accel  : linear acceleration (m/s^2)
    """

    # Convert rotation matrix to axis-angle rotation vector
    rot = R_scipy.from_matrix(R)
    rx, ry, rz = rot.as_rotvec()

    t = np.array(t).reshape(3,)   # ensure proper formatting

    # Build the UR pose: [x, y, z, rx, ry, rz]
    pose = [t[0], t[1], t[2], rx, ry, rz]

    # Send the moveL command
    rtde_c.moveL(pose, speed, accel)


# ==============================================
# Placeholder: robot movement logic
# ==============================================
def moveRobotToBlock(T_base_block):
    print("\n--- MOVE ROBOT TO BLOCK (placeholder) ---")
    # You will implement UR control here
    # e.g. send RTDE pose target or URScript movej/movel command
    t_gripper = T_base_block[:3, 3].copy()
    R_block = T_base_block[:3, :3].copy()
    
    # Rotate 180° around robot X-axis
    R_flip = R_scipy.from_euler('x', np.pi).as_matrix()

    R_gripper = R_block @ R_flip

    print("Moving gripper to: ", t_gripper)
    print("Gripper rotation matrix:\n", R_gripper)

    # Move to jenga block
    moveL(t_gripper, R_gripper)

    time.sleep(5)

    # Move back again (testing)
    moveL(np.array([0, -0.4, 0.4]), R_gripper)
    print("--- DONE ---")


# ==============================================
# Function: Pickup next Jenga block (ID 1 → up)
# ==============================================
def pickupNewBlock(block_poses_dict):
    global next_block_id

    if next_block_id not in block_poses_dict:
        print(f"Block with marker ID {next_block_id} not visible yet.")
        return False

    T_base_block = block_poses_dict[next_block_id]
    moveRobotToBlock(T_base_block)

    print(f"\nPicked block ID {next_block_id}!")
    next_block_id += 1
    return True


# ==============================================
# Main Loop
# ==============================================
print("Running. Press 'p' to pick next block, 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Dictionary to store block transforms for pickup function
    block_poses_in_base = {}

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for idx, marker_id in enumerate(ids.flatten()):
            rvec, tvec = None, None

            # Use different marker size for calibration marker vs block markers
            if marker_id == 0:
                size = marker_size_calibration
            else:
                size = marker_size_block

            # If a size is wrong, pose estimation breaks, so this is important
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                [corners[idx]],
                size,
                camera_matrix,
                dist_coeffs
            )

            rvec = rvecs[0]
            tvec = tvecs[0]

            T_cam_marker = rvec_tvec_to_T(rvec, tvec)

            # --------------------------
            # Compute camera extrinsics using marker 0
            # --------------------------
            if marker_id == 0 and T_base_cam is None:
                T_base_cam = T_base_marker @ np.linalg.inv(T_cam_marker)
                print("\n=== CAMERA POSE COMPUTED ===")
                print(T_base_cam)
                print("============================")

            # --------------------------
            # Compute block pose (if camera is calibrated)
            # --------------------------
            if T_base_cam is not None:
                T_base_block = T_base_cam @ T_cam_marker

                # Store for pickup
                if marker_id != 0:
                    block_poses_in_base[marker_id] = T_base_block

                # Draw axis for visualization
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

    # --------------------------
    # Listen for key commands
    # --------------------------
    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        break

    if key == ord('p'):  # Pick next block
        pickupNewBlock(block_poses_in_base)

    cv2.imshow("ArUco Block Detection + Robot Mapping", frame)

cap.release()
cv2.destroyAllWindows()
