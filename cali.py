from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs

import argparse
import cv2
import numpy as np


def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    parser.set_defaults(
    robot_ip="192.168.1.150",
    plotter=False,
    visualizer=False,
    gripper="onrobot",    
    )
    parser.description "Jenga builder calibration"
    parser.getClikArgs(parser)
    return parser.parse_args()



# ============================
# Helper Functions
# ============================

def rvec_tvec_to_matrix(rvec, tvec):
    """Convert OpenCV rvec/tvec to a 4x4 transformation matrix."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T

def invert_transform(T):
    """Invert a 4x4 homogeneous transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv

def detect_aruco_pose(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length):
    """
    Detects ArUco marker and returns pose matrix.
    Returns (T_camera_marker, marker_id) or (None, None) if not found.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None

    # Estimate pose for each detected marker
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_length, camera_matrix, dist_coeffs
    )
    
    # Return the FIRST marker for simplicity
    rvec = rvecs[0].reshape(3)
    tvec = tvecs[0].reshape(3)
    marker_id = int(ids[0][0])

    T = rvec_tvec_to_matrix(rvec, tvec)
    return T, marker_id


# ============================
# MAIN WORKFLOW
# ============================

def main():
    args = get_args()
    robot = getRobotFromArgs(args)
    robot._step()
    robot.setFreedrive()
    # ----------------------------
    # Load camera calibration
    # ----------------------------
    # Replace with your own values:
    camera_matrix = np.array([[615, 0, 320],
                              [0, 615, 240],
                              [0,   0,   1]], dtype=np.float32)
    dist_coeffs = np.zeros(5)  # or your real distortion values

    # Marker size in meters
    marker_length = 0.0225  # 4 cm marker example

    # Choose ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()

    #Find the robot coord of the aruco
    print("Choose robot coord on fixed aruco code \n 'c' to capture current pose of robot (put over aruco) \n 'v' use fixed coord \n 'q' quit after choosing robot coord")
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            robot._step()
            T_w_e = np.array(robot.T_w_e)
            T_robot_offset = np.eye(4)
            T_robot_offset[:3,3] = np.array([0.0, 0.0, 0.17])
            #T_robot_offset[2,3] = 0.17 
            T_robot_marker_fixed = (T_w_e @ T_robot_offset)
            print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
        elif key == ord('v'):
            T_robot_marker_fixed = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1])
            T_robot_marker_fixed[:3, 3] = [0.45, -0.20, 0.02]  # fill in with real measured values
            print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
        elif key = ord('q'):
            print("Done with robot coord, quitting")
            break
    robot.unSetFreedrive()
    # Rotation can also be added if needed.

    # Start camera
    cap = cv2.VideoCapture(0)

    print("Looking for FIXED aruco marker...")

    T_robot_camera = None

    # Step 1: Detect FIXED marker to compute camera pose
    while T_robot_camera is None:
        ret, frame = cap.read()
        if not ret:
            continue

        T_camera_marker_fixed, marker_id = detect_aruco_pose(
            frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length
        )

        if marker_id == 0:  # assuming fixed marker has ID=0
            print("Found fixed marker. Calibrating camera pose...")
            T_robot_camera = T_robot_marker_fixed @ invert_transform(T_camera_marker_fixed)
            print("T_robot_camera =\n", T_robot_camera)

        cv2.imshow("Calibration", frame)
        if cv2.waitKey(1) == ord('q'):
            break

    print("Camera pose calibrated.")
    print("Now detecting other markers...")

    # ----------------------------
    # Step 2: Detect ANY marker and convert to robot frame
    # ----------------------------
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        T_camera_marker, marker_id = detect_aruco_pose(
            frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length
        )

        if T_camera_marker is not None and marker_id != 0:
            # Convert marker pose to robot frame
            T_robot_marker = T_robot_camera @ T_camera_marker
            pos_robot = T_robot_marker[:3, 3]

            print(f"Marker {marker_id} in ROBOT frame:", pos_robot)
            print("Press 'y' to goto aruco point")
            if cv2.waitKey(1) == ord('y'):
                # INSERT YOUR ROBOT CONTROL HERE:
                robot_rot = np.array([1, 0, 0], [0, -1, 0], [0, 0, -1]])
                T_marker_tcp = np.eye(4)
                T_marker_tcp[:3, :3] = robot_rot
                T_marker_tcp[2,3] = 0.15 #Detta kanske är bra
                T_w_goal = T_robot_marker @ T_marker_tcp                
                moveL(args, robot, T_w_goal)
        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

