import cv2
import cv2.aruco as aruco
import numpy as np
import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R_scipy
import xmlrpc.client
import time
from video_stream import VideoStream

ROBOT_IP = "192.168.1.150"
GRIPPER_IP = "192.168.1.1"

rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

cb = xmlrpc.client.ServerProxy(f"http://{GRIPPER_IP}:41414/")

data = np.load("camera_charuco_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

url = "http://klasthorgren:video123@10.37.196.204:8081/video"
cap = cv2.VideoCapture(url)
#vs = VideoStream(url).start()
#time.sleep(0.2)   # allow a couple frames to arrive

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

marker_size_calibration = 0.0748
marker_size_block = 0.0225

block_width = 0.028
block_height = 0.015

tower_origin_base = np.array([0.1, -0.3, 0.0])

base_marker_pos = np.array([0.400, -0.300, 0.000])  # example
base_marker_rot = np.array([[-1, 0, 0],
                            [0, -1, 0],
                            [0, 0, 1]])

T_base_marker = np.eye(4)
T_base_marker[:3, :3] = base_marker_rot
T_base_marker[:3, 3] = base_marker_pos

even_rotation = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
odd_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

# Will be computed from marker ID 0
T_base_cam = None

# Keeps track of which block (ID) to pick next
next_block_id = 1

def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

def open_gripper():
    """Fully open the 2FG7 using maximum external width."""
    max_width = cb.twofg_get_max_external_width(0)
    cb.twofg_grip_external(0, max_width, 140, 50)   # (id, width, force, speed)
    time.sleep(1)

def close_gripper():
    """Fully close the 2FG7."""
    cb.twofg_grip_external(0, 0.0, 140, 50)
    time.sleep(1)

def moveL(t, R, speed=0.2, accel=0.4):
    """
    Move UR robot linearly (moveL) with translation vector t and rotation matrix R.
    """

    rot = R_scipy.from_matrix(R)
    rx, ry, rz = rot.as_rotvec()

    t = np.array(t).reshape(3,)

    pose = [t[0], t[1], t[2], rx, ry, rz]

    rtde_c.moveL(pose, speed, accel)

def pick_up_block_from_vision(T_base_block):
    open_gripper()
    # Approach from above
    pickup_height_offset = 0.05  # approach 10 cm above detected block
    t_above = T_base_block[:3, 3].copy()
    t_above[2] += pickup_height_offset

    R_block = T_base_block[:3, :3]
    R_flip = R_scipy.from_euler('x', np.pi).as_matrix()      # Flip down
    R_z90 = R_scipy.from_euler('z', np.pi/2).as_matrix()     # Rotate around Z
    R_gripper = R_block @ R_flip @ R_z90

    # Move above block
    moveL(t_above, R_gripper)

    #t_pickup = t_above.copy()
    #t_pickup[2] -= 0.118
    rtde_c.moveUntilContact([0, 0, -0.02, 0, 0, 0])

    #moveL(t_pickup, R_gripper)
    
    close_gripper()
    time.sleep(1)

    # Move back up
    moveL(t_above, R_gripper)

def place_block(current_block, current_layer):
    is_even = (current_layer % 2 == 0)

    # Create transform
    T_place = np.eye(4)

    # Base tower reference
    T_place[:3, 3] = tower_origin_base.copy()

    # Horizontal placement depending on orientation
    if is_even:
        # Blocks along X direction → shift X
        T_place[0, 3] += block_width * (1 - current_block)
        T_place[:3, :3] = even_rotation
    else:
        # Blocks along Y direction → shift Y
        T_place[1, 3] -= block_width * (current_block - 1)
        T_place[:3, :3] = odd_rotation

    T_place[2, 3] = current_layer * block_height + 0.05

    moveL(T_place[:3, 3], T_place[:3, :3])

    rtde_c.moveUntilContact([0, 0, -0.02, 0, 0, 0])

    open_gripper()

    time.sleep(1)

    if current_block == 2:
        new_pose = T_place.copy()
        
        # get z value
        new_pose[2, 3] = rtde_r.getActualTCPPose()[2]

        # higher 4 cm
        T_place[2, 3] = new_pose[2, 3] + 0.04
        moveL(T_place[:3, 3], T_place[:3, :3])

        # move to x-y orgin and turn gripper
        T_place[:2, 3] = tower_origin_base[:2]
        if is_even:
            T_place[:3, :3] = odd_rotation
        else:
            T_place[:3, :3] = even_rotation

        moveL(T_place[:3, 3], T_place[:3, :3])

        # lower again
        T_place[2, 3] = new_pose[2, 3]
        moveL(T_place[:3, 3], T_place[:3, :3])

        close_gripper()
        time.sleep(1)
        open_gripper()

    # raise end effector
    T_place[2, 3] += 0.10
    moveL(T_place[:3, 3], T_place[:3, :3])

def read_latest(cap, max_clear=5):
    # Read and drop max_clear frames at most
    for _ in range(max_clear):
        grabbed = cap.grab()
        if not grabbed:
            break
    ret, frame = cap.read()
    return frame

def disconnect_camera(cap):
    try:
        cap.release()
    except:
        pass
    time.sleep(0.2)

def reconnect_camera(url):
    cap = cv2.VideoCapture(url)
    time.sleep(0.3)   # wait for actual frames to arrive
    return cap

while True:
    frame = read_latest(cap)
    if frame is None:
        cap.read()
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

    if key == ord('p'):
        if next_block_id in block_poses_in_base:
            T_base_block = block_poses_in_base[next_block_id]
            disconnect_camera(cap)
            pick_up_block_from_vision(T_base_block)  # NEW pickup
            place_block((next_block_id-1) % 3, (next_block_id-1) // 3)  # use build logic
            cap = reconnect_camera(url)
            next_block_id += 1
        else:
            print("Block not visible!")

    cv2.imshow("ArUco Block Detection + Robot Mapping", frame)

cap.release()
cv2.destroyAllWindows()
#vs.stop()