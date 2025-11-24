from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import pinocchio as pin

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
    np.set_printoptions(suppress=True)
    #ROBOT INITS
    args = get_args()
    robot = getRobotFromArgs(args)
    robot._step()
    robot.setFreedrive()
    
    #ARUCO INITS
    # Marker size in meters
    marker_length = 0.008
    marker_length_block = 0.0225
    marker_length_fixed = 0.1 
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
    #CALIBRATION OF CAMERA
    try:
        #CAMERA INITS
        url = "http://klasthorgren:video123@10.8.58.61:8081/video"
        cap = cv2.VideoCapture(url)

        all_corners = []
        all_ids = []
        img_size = None
        
        last_time = 0 
        print("Calibrate camera, move the board slowly. Press 'q' to finish.")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("no ret captured, breaking")
                break
            now = time.time()
            if now - last_time > 0.1:
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
                        print("Corners detected this frame:", len(charuco_ids))
                        if len(charuco_ids) > 4:
                            all_corners.append(charuco_corners)
                            all_ids.append(charuco_ids)
                            img_size = gray.shape[::-1]

            cv2.imshow("Calibration", frame)
            #time.sleep(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("User pressed 'q', breaking.")
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
     
        #CALIBRATE ROBOT
        #Find the robot coord of the aruco
        print("\nChoose robot coord on fixed aruco code \n 'c' to capture current pose of robot (put over aruco) \n 'v' use fixed coord \n 'q' quit after choosing robot coord")
        T_robot_marker_fixed = None
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                robot._step()
                T_robot_marker_fixed = np.array(robot.T_w_e)
                #T_w_e = np.array(robot.T_w_e)
                #T_robot_offset = np.eye(4)
                #T_robot_offset[:3,3] = np.array([0.0, 0.0, 0.152])
                #T_robot_marker_fixed = (T_w_e @ T_robot_offset)
                print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
            elif key == ord('v'):                
                T_robot_marker_fixed = np.array([[0.99991516, -0.01210937, 0.00479953, 0.48303724], 
                                                 [-0.01207788, -0.99990569, -0.00653728, -0.3998456], 
                                                 [0.00487824, 0.00647876, -0.99996711, 0.15247327], 
                                                 [0, 0, 0, 1]])
                print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
            elif key == ord('q'):
                print("Done with robot coord, quitting")
                break
            cv2.imshow("Calibration", frame)
        robot.unSetFreedrive()

        print("Looking for FIXED aruco marker")
        T_robot_camera = None
        # Step 1: Detect FIXED marker to compute camera pose
        while T_robot_camera is None:
            ret, frame = cap.read()
            if not ret:
                continue

            T_camera_marker_fixed, marker_id = detect_aruco_pose(
                frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length_fixed
            )

            if marker_id == 1:  # assuming fixed marker has ID=1
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
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            if ids is None:
                continue

            aruco.drawDetectedMarkers(frame, corners, ids)
            
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_length_block, camera_matrix, dist_coeffs)
            
            rvec = rvecs[0].reshape(3)
            tvec = tvecs[0].reshape(3)
            marker_id = int(ids[0][0])
            T_camera_marker = rvec_tvec_to_matrix(rvec, tvec)
            
            for rvec, tvec in zip(rvecs, tvecs):
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)
                
            if T_camera_marker is not None and marker_id != 1:
                
                # Convert marker pose to robot frame
                T_robot_marker = T_robot_camera @ T_camera_marker
                pos_robot = T_robot_marker[:3, 3]

                print(f"Marker {marker_id} in ROBOT frame:", pos_robot)
                print("Press 'y' to goto aruco point")
                if cv2.waitKey(1) == ord('y'):
                    # INSERT YOUR ROBOT CONTROL HERE:
                    robot_rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                    pos_robot = pos_robot + np.array([0, 0, 0.20])
                    T_w_goal = pin.SE3(robot_rot, pos_robot)                
                    moveL(args, robot, T_w_goal)
                    robot.stopRobot()
            cv2.imshow("Calibration", frame)
            if cv2.waitKey(1) == ord('q'):
                break
       
        #Save to file 
        np.savez("camera_charuco_calibration.npz",
                 camera_matrix=camera_matrix,
                 dist_coeffs=dist_coeffs)
         
        np.savez("robot_camera_calibration.npz",
                T_robot_camera)

    except KeyboardInterrupt:
        print("Interrupted by user.") 
    
    #Cleanup
    cap.release()
    cv2.destroyAllWindows()
    if args.visualizer:
        robot.killManipulatorVisualizer()

    if args.save_log:
        robot._log_manager.saveLog()

    if args.real:
        robot.stopRobot()

if __name__ == "__main__":
    main()

