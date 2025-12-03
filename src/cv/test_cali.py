#!/usr/bin/env python3
"""
Camera <-> Robot calibration and block detection (fixed version).

- Collect several robot poses with the end-effector over a known fixed ArUco marker.
- Collect corresponding ArUco poses in camera coordinates.
- Run cv2.calibrateHandEye to compute camera->gripper transform.
- Invert and chain to compute robot->camera transform.
- Use robot->camera to convert subsequently-detected block markers into robot frame.

Run:
    python3 camera_robot_calibration_fixed.py

Notes:
- marker lengths must be in meters.
- ensure your camera feed URL is correct.
- this script expects the smc robot interface that provides robot.T_w_e (4x4) as in your original script.
"""

from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL

import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import pinocchio as pin

# -------------------------
# Helpers
# -------------------------
def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    parser.set_defaults(
        robot_ip="192.168.1.150",
        plotter=False,
        visualizer=False,
        gripper="onrobot",
    )
    parser.description = "Jenga builder calibration (fixed version)"
    parser = getClikArgs(parser)
    return parser.parse_args()


def rvec_tvec_to_matrix(rvec_or_R, tvec):
    """
    Convert an OpenCV rvec (Rodrigues vector) + tvec OR a 3x3 rotation matrix + tvec
    into a 4x4 homogeneous transform (numpy array).
    Accepts:
      - rvec_or_R shape (3,) or (3,1)  -> treated as Rodrigues rotation vector
      - rvec_or_R shape (3,3)          -> treated as rotation matrix
    tvec may be (3,) or (3,1).
    """
    r = np.array(rvec_or_R)
    if r.shape == (3,) or r.shape == (3, 1) or r.size == 3:
        R_mat, _ = cv2.Rodrigues(r.reshape(3))
    elif r.shape == (3, 3):
        R_mat = r
    else:
        raise ValueError("rvec_or_R must be (3,) or (3,3). Got shape: %r" % (r.shape,))
    t = np.array(tvec).reshape(3)
    T = np.eye(4)
    T[:3, :3] = R_mat
    T[:3, 3] = t
    return T


def invert_transform(T):
    """Invert a 4x4 homogeneous transform."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def detect_aruco_pose(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length, wanted_id=None):
    """
    Detect ArUco markers and return pose of the marker with `wanted_id`.
    If wanted_id is None, returns the first detected marker (T, id).
    Returns (T_camera_marker, marker_id) or (None, None) if not found.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None

    # Estimate pose for all detected markers
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

    # iterate through detected markers, find wanted_id
    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if (wanted_id is not None) and (marker_id != wanted_id):
            continue
        rvec = rvecs[i].reshape(3)
        tvec = tvecs[i].reshape(3)
        T = rvec_tvec_to_matrix(rvec, tvec)
        return T, marker_id

    # wanted_id not found
    return None, None


def rotation_average_from_matrices(R_list):
    """
    Compute an average rotation from a list of rotation matrices using SVD method:
    find orthogonal matrix closest to sum(R_i).
    """
    M = np.zeros((3, 3))
    for R in R_list:
        M += R
    U, S, Vt = np.linalg.svd(M)
    R_avg = U @ Vt
    # ensure proper rotation (det = +1)
    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1
        R_avg = U @ Vt
    return R_avg


# -------------------------
# Main workflow
# -------------------------
def main():
    np.set_printoptions(suppress=True, precision=6)
    args = get_args()

    # -------------------------
    # Initialize robot
    # -------------------------
    print("Initializing robot (sim or real depending on args)")
    robot = getRobotFromArgs(args)
    robot._step()
    robot.setFreedrive()

    # -------------------------
    # ArUco & camera parameters
    # -------------------------
    # marker sizes (meters) - ensure these are correct for your markers
    marker_length_charuco = 0.008
    marker_length_block = 0.0225
    marker_length_fixed = 0.0563
    square_length = 0.012

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    board = aruco.CharucoBoard(size=(10, 7), squareLength=square_length, markerLength=marker_length_charuco, dictionary=aruco_dict)

    # -------------------------
    # Camera calibration (Charuco)
    # -------------------------
    try:
        # adjust this URL to your camera stream
        url = "http://klasthorgren:video123@10.15.116.64:8081/video"
        cap = cv2.VideoCapture(url)
        if not cap.isOpened():
            print("ERROR: cannot open camera stream at", url)
            return

        all_corners = []
        all_ids = []
        img_size = None

        # Option to load existing calibration
        new_charuco_calibration = True
        command = input("Do you want to calibrate using charuco, 'y'/'n'. Otherwise load file: ")
        if command.lower() == "n":
            data = np.load("camera_charuco_calibration.npz")
            camera_matrix = data["camera_matrix"]
            dist_coeffs = data["dist_coeffs"]
            new_charuco_calibration = False
            print("Loaded camera_charuco_calibration.npz")
        else:
            print("Collecting charuco images. Move the board slowly. Press 'q' to finish capture.")

        last_time = 0.0
        while new_charuco_calibration:
            ret, frame = cap.read()
            if not ret:
                continue
            now = time.time()
            # reduce rate of detections to 1Hz to save CPU and not add close-duplicate frames
            if now - last_time > 1.0:
                last_time = now
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict)
                if ids is not None:
                    _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(markerCorners=corners, markerIds=ids, image=gray, board=board)
                    if charuco_ids is not None and len(charuco_ids) > 4:
                        all_corners.append(charuco_corners)
                        all_ids.append(charuco_ids)
                        img_size = gray.shape[::-1]
                        print(f"Saved charuco frame: {len(charuco_ids)} corners")

            cv2.imshow("Calibration", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                if len(all_corners) < 3:
                    print("Need at least 3 good charuco frames. Continue capturing.")
                    continue
                break

        if new_charuco_calibration:
            print("Running charuco calibration...")
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(all_corners, all_ids, board, img_size, None, None)
            np.savez("camera_charuco_calibration.npz", camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
            print("Saved camera_charuco_calibration.npz")

        # -------------------------
        # Collect robot poses corresponding to fixed marker positions
        # -------------------------
        print("\nCollect robot-called poses over the fixed marker.")
        print("Controls: 'c' capture current robot pose (hand over marker), 'v' load example set, 'r' remove last, 'q' done when >=4")
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
                # read robot.T_w_e (4x4) as provided by your robot interface
                T_robot_marker_fixed = np.array(robot.T_w_e)
                print("Recorded raw T_robot_gripper (T_w_e):\n", T_robot_marker_fixed)

                # If you need to offset along tool z (for example to use a point below the flange),
                # apply the offset as a transform in the gripper frame.
                # Example: move 0.15 m along gripper z axis:
                tool_offset = 0.15  # meters
                # create transform representing an offset in gripper frame:
                T_offset = np.eye(4)
                T_offset[:3, 3] = np.array([0.0, 0.0, tool_offset])
                # compute T_robot_gripper_with_offset = T_robot_gripper * offset
                T_robot_gripper_with_offset = T_robot_marker_fixed @ T_offset
                print("T_robot_gripper_with_offset:\n", T_robot_gripper_with_offset)
                # store rotation and translation consistently
                R_robot_marker.append(T_robot_gripper_with_offset[:3, :3])
                t_robot_marker.append(T_robot_gripper_with_offset[:3, 3].reshape(3, 1))
                print(f"Saved sample. Now have {len(R_robot_marker)} samples.")
            elif key == ord('r'):
                if len(R_robot_marker) == 0:
                    print("Nothing to remove.")
                else:
                    R_robot_marker.pop()
                    t_robot_marker.pop()
                    print(f"Removed last. Now have {len(R_robot_marker)} samples.")
            elif key == ord('v'):
                # Example fixed coordinates (keep the shapes consistent)
                R_robot_marker.clear()
                t_robot_marker.clear()
                R_robot_marker.append(np.array([[-0.99999903, -0.00136024, 0.00029296],
                                               [-0.0013562, -0.9999096, -0.0133776],
                                               [0.0031113, 0.01337718, -0.99991047]]))
                t_robot_marker.append(np.array([0.51289175, -0.51103999, 0.00399951]).reshape(3, 1))
                R_robot_marker.append(np.array([[0.99998362, -0.00167552, 0.00547218],
                                               [-0.00146084, -0.99923803, -0.03900278],
                                               [0.00553336, 0.03899414, -0.99922412]]))
                t_robot_marker.append(np.array([0.38539166, -0.51198535, 0.00405456]).reshape(3, 1))
                R_robot_marker.append(np.array([[0.9999866, -0.00415944, 0.00308346],
                                               [-0.00411011, -0.99986615, -0.01583645],
                                               [0.00314892, 0.01582356, -0.99986984]]))
                t_robot_marker.append(np.array([0.51216683, -0.29065349, 0.00109551]).reshape(3, 1))
                R_robot_marker.append(np.array([[0.99990529, -0.01131563, 0.00783321],
                                               [-0.01115125, -0.99972314, -0.02071954],
                                               [0.0080655, 0.02063022, -0.99975464]]))
                t_robot_marker.append(np.array([0.38274475, -0.29115342, 0.00043638]).reshape(3, 1))
                print("Loaded example robot samples.")
            elif key == ord('q'):
                if len(R_robot_marker) < 4:
                    print(f"Need at least 4 robot samples, currently have {len(R_robot_marker)}.")
                    continue
                print("Done collecting robot samples.")
                break

            cv2.imshow("Calibration", frame)

        robot.unSetFreedrive()
        print("Collected robot samples (R, t):")
        for i, (R, t) in enumerate(zip(R_robot_marker, t_robot_marker)):
            print(f"[{i}] R=\n{R}\n t={t.ravel()}")

        # -------------------------
        # Collect corresponding camera poses for the FIXED marker(s)
        # -------------------------
        print("\nLooking for fixed ArUco markers (IDs 0..3). Press 'q' when finished capture.")
        R_camera_marker = []
        t_camera_marker = []
        cur_id = 0
        while cur_id < 4:
            ret, frame = cap.read()
            if not ret:
                continue

            # detect specific wanted id
            T_cam_marker, found_id = detect_aruco_pose(frame, aruco_dict, aruco_params, camera_matrix, dist_coeffs, marker_length_fixed, wanted_id=cur_id)
            if T_cam_marker is not None and found_id == cur_id:
                R_camera_marker.append(T_cam_marker[:3, :3])
                t_camera_marker.append(T_cam_marker[:3, 3].reshape(3, 1))
                print(f"Found fixed marker id {cur_id} and saved camera pose.")
                cur_id += 1

            # show frame + detected markers for debug (draw all)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow("Calibration", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                if len(R_camera_marker) < 4:
                    print(f"Need at least 4 camera samples, currently have {len(R_camera_marker)}. Continue capturing.")
                    continue
                break

        print("Collected camera samples for fixed markers (R, t):")
        for i, (R, t) in enumerate(zip(R_camera_marker, t_camera_marker)):
            print(f"[{i}] R=\n{R}\n t={t.ravel()}")

        # -------------------------
        # Run calibrateHandEye
        # -------------------------
        # convert lists to proper numpy arrays / shapes (cv2 expects lists of arrays)
        R_robot_list = [np.asarray(R).astype(np.float64) for R in R_robot_marker]
        t_robot_list = [np.asarray(t).reshape(3, 1).astype(np.float64) for t in t_robot_marker]
        R_cam_list = [np.asarray(R).astype(np.float64) for R in R_camera_marker]
        t_cam_list = [np.asarray(t).reshape(3, 1).astype(np.float64) for t in t_camera_marker]

        print("Running cv2.calibrateHandEye(...)")
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_robot_list, t_robot_list, R_cam_list, t_cam_list, method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        # R_cam2gripper is rotation from camera -> gripper, t_cam2gripper is translation (3x1)

        # Build homogeneous T_cam2gripper
        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = np.array(t_cam2gripper).reshape(3)
        print("T_cam2gripper:\n", T_cam2gripper)

        # invert -> T_gripper2cam
        T_gripper2cam = invert_transform(T_cam2gripper)
        print("T_gripper2cam (inv):\n", T_gripper2cam)

        # Option: compute T_robot_camera for each sample and average them (robust)
        T_robot_camera_list = []
        for (Rr, tr) in zip(R_robot_marker, t_robot_marker):
            T_robot_gripper = np.eye(4)
            T_robot_gripper[:3, :3] = Rr
            T_robot_gripper[:3, 3] = np.array(tr).reshape(3)
            T_robot_camera_i = T_robot_gripper @ T_gripper2cam
            T_robot_camera_list.append(T_robot_camera_i)

        # average rotation using SVD method and average translation
        R_sum = sum(T[:3, :3] for T in T_robot_camera_list)  # elementwise sum
        R_avg = rotation_average_from_matrices([T[:3, :3] for T in T_robot_camera_list])
        t_avg = np.mean([T[:3, 3] for T in T_robot_camera_list], axis=0)
        T_robot_camera = np.eye(4)
        T_robot_camera[:3, :3] = R_avg
        T_robot_camera[:3, 3] = t_avg
        print("Estimated T_robot_camera (averaged):\n", T_robot_camera)

        # Save calibration
        np.savez("robot_camera_calibration.npz", T_robot_camera=T_robot_camera, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
        print("Saved robot_camera_calibration.npz")

        print("Now detecting block markers and converting to robot frame. Press 'y' to move to a printed marker (>= id 4). Press 'q' to quit.")
        # -------------------------
        # Step 2: Detect any block markers and convert to robot frame
        # -------------------------
        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            if ids is None:
                cv2.imshow("Calibration", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length_block, camera_matrix, dist_coeffs)

            # iterate all detections (not only index 0)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                rvec = rvecs[i].reshape(3)
                tvec = tvecs[i].reshape(3)
                T_camera_marker = rvec_tvec_to_matrix(rvec, tvec)
                # draw frame axes for each detection (visual)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

                if marker_id >= 4:
                    # convert to robot frame
                    T_robot_marker = T_robot_camera @ T_camera_marker
                    pos_robot = T_robot_marker[:3, 3]
                    print(f"Marker {marker_id} in ROBOT frame: {pos_robot}")

                    # if user presses 'y', move to goal (with a z offset safety)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('y'):
                        robot_rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                        pos_robot_safe = pos_robot + np.array([0, 0, 0.20])  # approach height
                        T_w_goal = pin.SE3(robot_rot, pos_robot_safe)
                        moveL(args, robot, T_w_goal)
                        robot.stopRobot()

            cv2.imshow("Calibration", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # cleanup
        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
        if args.visualizer:
            robot.killManipulatorVisualizer()
        if args.save_log:
            robot._log_manager.saveLog()
        if args.real:
            robot.stopRobot()


if __name__ == "__main__":
    main()
