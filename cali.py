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
    T = np.eye(4)
    r = np.array(rvec)
    if r.shape == (3,) or r.shape == (3,1) or r.size == 3:
        R, _ = cv2.Rodrigues(r.reshape(3))
    elif r.shape == (3,3):
        R = r
    else:
        print("rvec is wrong shape, must be  (3,) or (3,3).")
        return None
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).reshape(3)
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
    marker_length_charuco = 0.008
    marker_length_block = 0.0225
    marker_length_fixed = 0.0563 
    square_length = 0.012
    # Choose ArUco dictionary
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    board = aruco.CharucoBoard(
        size=(10,7),
        squareLength=square_length,
        markerLength=marker_length_charuco,
        dictionary=aruco_dict)


    #CALIBRATION OF CAMERA
    try:
        #CAMERA INITS
        url = "http://klasthorgren:video123@10.15.116.64:8081/video"
        cap = cv2.VideoCapture(url)

        all_corners = []
        all_ids = []
        img_size = None
        

        #--------------------CHARUCO CALIBRATION --------------------
        new_charuco_calibration = True
        command = input("Do you want to calibrate using charuco, 'y'/'n'. Otherwise load file.")
        if command.lower() == "n":
            data = np.load("camera_charuco_calibration.npz")
            camera_matrix = data["camera_matrix"]
            dist_coeffs = data["dist_coeffs"]
            new_charuco_calibration = False

        print("Calibrate camera, move the board slowly. Press 'q' to finish.")
        last_time = 0 
        while new_charuco_calibration:
            ret, frame = cap.read()
            if not ret:
                print("no ret captured, breaking")
                break
            now = time.time()
            if now - last_time > 1:
                last_time = now 
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)

                if ids is not None:
                    _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                        markerCorners=corners,
                        markerIds=ids,
                        image=gray,
                        board=board)
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
        if new_charuco_calibration: 
            #Calibrate
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                all_corners,
                all_ids,
                board,
                img_size,
                None,
                None)
            #Save to file 
            np.savez("camera_charuco_calibration.npz",
                 camera_matrix=camera_matrix,
                 dist_coeffs=dist_coeffs)
        

 
        #-------------------- CALIBRATE ROBOT --------------------
        #Find the robot coord of the aruco
        print("\nChoose robot coord on fixed aruco code \n 'c' to capture current pose of robot (put over aruco) \n 'v' use fixed coord \n 'r' to remove last point \n 'q' quit after choosing robot coord")
        T_robot_marker_fixed = None
        R_robot_marker = []
        t_robot_marker = []    
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            key = cv2.waitKey(1) & 0xFF
            if key == ord('c'):
                robot._step()
                T_robot_marker_fixed = np.array(robot.T_w_e)
                print("T_robot_marker_fixed=\n", T_robot_marker_fixed)
                t_end_effector = T_robot_marker_fixed @ np.array([0, 0, 0.15, 1])
                T_robot_marker_fixed[:3, 3] = t_end_effector[:3]
                R_robot_marker.append(T_robot_marker_fixed[:3, :3])
                t_robot_marker.append(T_robot_marker_fixed[:3, 3].reshape(3,1))
                print("T_robot_marker_fixed_withoffset=\n", T_robot_marker_fixed)
                print(f"Saved point to array. Need 4 points, currently have {len(R_robot_marker)}.")
            elif key == ord('r'):
                if len(R_robot_marker) < 1:
                    print("Nothing to remove")
                    continue
                print("Removed last added point, now {len(R_robot_marker) points saved.")
                R_robot_marker.pop()
                t_robot_marker.pop()
            elif key == ord('v'):     
                R_robot_marker.clear()
                t_robot_marker.clear()
                R_robot_marker.append(np.array([[-0.99999903, -0.00136024, 0.00029296], 
                                                [-0.0013562, -0.9999096, -0.0133776], [0.0031113, 0.01337718, -0.99991047]]))
                t_robot_marker.append(np.array([0.51289175, -0.51103999, 0.00399951]).reshape(3,1))           
                R_robot_marker.append(np.array([[0.99998362, -0.00167552, 0.00547218], 
                                                [-0.00146084, -0.99923803, -0.03900278], [0.00553336, 0.03899414, -0.99922412]]))
                t_robot_marker.append(np.array([0.38539166, -0.51198535, 0.00405456]).reshape(3,1))           
                R_robot_marker.append(np.array([[0.9999866, -0.00415944, 0.00308346], 
                                                [-0.00411011, -0.99986615, -0.01583645], [0.00314892, 0.01582356, -0.99986984]]))
                t_robot_marker.append(np.array([0.51216683, -0.29065349, 0.00109551]).reshape(3,1))           
                R_robot_marker.append(np.array([[0.99990529, -0.01131563, 0.00783321], 
                                                [-0.01115125, -0.99972314, -0.02071954], [0.0080655, 0.02063022, -0.99975464]]))
                t_robot_marker.append(np.array([0.38274475, -0.29115342, 0.00043638]).reshape(3,1))           
                print("Cleared the arrays and added fixed coordinates.")
            elif key == ord('q'):
                if len(R_robot_marker) < 4:
                    print(f"Need atleast 4 points, currently have {len(R_robot_marker)}.")
                    continue
                print("Done with robot coord, quitting")
                break
            cv2.imshow("Calibration", frame)
        robot.unSetFreedrive()
        print("Robot to marker pose, R = , t = .")
        for r, t in zip(R_robot_marker, t_robot_marker):
            print(f"R = {r} \nt = {t}")

        #-------------------- CALIBRATE FIXED ARUCO --------------------
        print("Looking for FIXED aruco marker, press 'q' after finding 4 markers.")
        T_robot_camera = None
        R_camera_marker = []
        t_camera_marker = []
        cur_id = 0
        # Step 1: Detect FIXED marker to compute camera pose
        while T_robot_camera is None:
            ret, frame = cap.read()
            if not ret:
                continue

            T_camera_marker_fixed, marker_id = detect_aruco_pose(
                frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length_fixed)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if ids is not None:
                # Estimate pose for each detected marker
                ids = ids.flatten()
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, marker_length_fixed, camera_matrix, dist_coeffs)
                if (cur_id in ids) and cur_id < 4:
                    i = np.where(ids == cur_id)[0][0]
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    R, _ = cv2.Rodrigues(rvec)   
                    R_camera_marker.append(R)
                    t_camera_marker.append(tvec.reshape(3,1))
                    print(f"Found fixed marker {cur_id} with R={R} and t={tvec}, adding to array.")
                    cur_id += 1
                cv2.imshow("Calibration", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                if len(R_camera_marker) < 3:
                    print("Currently have {len(R_camera_marker)}. Need 4 points.")
                    continue
                break
        #Calibrate Robot to Camera frame
        R_robot_camera, t_robot_camera = cv2.calibrateHandEye(
            R_robot_marker, 
            t_robot_marker, 
            R_camera_marker, 
            t_camera_marker,
            method=cv2.CALIB_HAND_EYE_DANIILIDIS
            )
        T_robot_camera = rvec_tvec_to_matrix(R_robot_camera, t_robot_camera)
        #Save to file 
        np.savez("robot_camera_calibration.npz",
                T_robot_camera)

        print("Camera pose calibrated.")
        print("T_robot_camera = \n", T_robot_camera)
        
        #-------------------- TESTING --------------------
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
                
            if T_camera_marker is not None and (marker_id >= 4):
                
                # Convert marker pose to robot frame
                T_robot_marker = T_robot_camera @ T_camera_marker
                pos_robot = T_robot_marker[:3, 3]

                print(f"Marker {marker_id} in ROBOT frame:", pos_robot)
                print("Press 'y' to goto aruco point")
                if cv2.waitKey(1) == ord('y'):
                    robot_rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                    pos_robot = pos_robot + np.array([0, 0, 0.20])
                    T_w_goal = pin.SE3(robot_rot, pos_robot)                
                    moveL(args, robot, T_w_goal)
                    robot.stopRobot()
            cv2.imshow("Calibration", frame)
            if cv2.waitKey(1) == ord('q'):
                break
    

   
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

