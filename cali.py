from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs

import argparse
import cv2
from cv2 import aruco
import numpy as np
import time


def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    parser.set_defaults(
    robot_ip="192.168.1.150",
    plotter=False,
    visualizer=False,
    gripper="onrobot",    
    )
    parser.description = "Jenga builder calibration"
    parser = getClikArgs(parser)
    return parser.parse_args()



#Helper Functions
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

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None

    # Estimate pose for each detected marker
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, marker_length, camera_matrix, dist_coeffs
    )
    
    # Return the FIRST marker for simplicity
    rvec = rvecs[0].reshape(3)
    tvec = tvecs[0].reshape(3)
    marker_id = int(ids[0][0])

    T = rvec_tvec_to_matrix(rvec, tvec)
    return T, marker_id


#MAIN WORKFLOW
def main():
    #ROBOT INITS
    args = get_args()
    robot = getRobotFromArgs(args)
    robot._step()
    robot.setFreedrive()
    
    #ARUCO INITS
    # Marker size in meters
    marker_length = 0.008
    marker_length_block = 0.0225
    square_length = 0.012
    # Choose ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    board = aruco.CharucoBoard(
        size=(10,7),
        squareLength=square_length,
        markerLength=marker_length,
        dictionary=aruco_dict
    )
    
    #CAMERA INITS
    url = "http://klasthorgren:video123@10.8.58.61:8081/video"
    cap = cv2.VideoCapture(url)

    all_corners = []
    all_ids = []
    img_size = None
    
    #CALIBRATION OF CAMERA
    print("Calibrate camera, move the board slowly. Press 'q' to finish.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("no ret captured, breaking")
            break

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
                print("Corners detected this frame:", len(charuco_ids))
                if len(charuco_ids) > 4:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    img_size = gray.shape[::-1]

        time.sleep(1)

        cv2.imshow("Calibration", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

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
       
 
    #CALIBRATE ROBOT
    #Find the robot coord of the aruco
    print("Choose robot coord on fixed aruco code \n 'c' to capture current pose of robot (put over aruco) \n 'v' use fixed coord \n 'q' quit after choosing robot coord")
    T_robot_marker_fixed = None
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            robot._step()
            T_w_e = np.array(robot.T_w_e)
            T_robot_offset = np.eye(4)
            T_robot_offset[:3,3] = np.array([0.0, 0.0, 0.17])
            T_robot_marker_fixed = (T_w_e @ T_robot_offset)
            print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
        elif key == ord('v'):
            T_robot_marker_fixed = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            T_robot_marker_fixed[:3, 3] = [0.45, -0.20, 0.02]  # fill in with real measured values
            print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
        elif key == ord('q'):
            print("Done with robot coord, quitting")
            break
    robot.unSetFreedrive()
    # Rotation can also be added if needed.

    print("Looking for FIXED aruco marker")
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
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Camera pose calibrated.")
    print("Now detecting other markers.")

    #Step 2: Detect ANY marker and convert to robot frame
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        T_camera_marker, marker_id = detect_aruco_pose(
            frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length_block
        )    
        if T_camera_marker is not None and marker_id != 0:
            # Convert marker pose to robot frame
            T_robot_marker = T_robot_camera @ T_camera_marker
            pos_robot = T_robot_marker[:3, 3]

            print(f"Marker {marker_id} in ROBOT frame:", pos_robot)
            print("Press 'y' to goto aruco point")
            if cv2.waitKey(1) == ord('y'):
                # INSERT YOUR ROBOT CONTROL HERE:
                robot_rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
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
    
    np.savez("robot_camera_calibration.npz",
            T_robot_camera)
     

if __name__ == "__main__":
    main()

