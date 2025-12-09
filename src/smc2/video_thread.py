# video_thread.py
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import mediapipe as mp
from shared_state import pause_event

# Keep identical constants/params as original script
url = "http://klasthorgren:video123@10.37.196.204:8081/video"

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

# Load camera calibration (same as before)
data = np.load("camera_charuco_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

def rvec_tvec_to_T(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T

class VideoWorker:
    def __init__(self, cmd_queue, resp_queue):
        self.cmd_queue = cmd_queue
        self.resp_queue = resp_queue
        self._running = True
        self.next_block_id = 1
        self.T_base_cam = None
        self.send_command = True

        # ---- Faster Mediapipe config ----
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            model_complexity=0
        )

        # initialize capture (low latency settings)
        self.cap = cv2.VideoCapture(url)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.frame_count = 0
        time.sleep(0.2)

    def stop(self):
        self._running = False

    def run(self):
        global aruco_dict, parameters

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # ----- Hand detection (every frame, lightweight model) -----
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hand_result = self.mp_hands.process(rgb)

            if hand_result.multi_hand_landmarks:
                pause_event.set()
                cv2.putText(
                    frame,
                    "Hand detected!",
                    (2500, 300),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    5,
                    (0, 0, 255),
                    4
                )
            else:
                pause_event.clear()

            corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            block_poses_in_base = {}

            # Use last known good markers for display and pose
            if ids is not None:
                #aruco.drawDetectedMarkers(frame, last_corners, last_ids)

                for idx, marker_id in enumerate(ids.flatten()):

                    # Decide if this marker should be processed
                    should_process = (
                        (marker_id == 0 and self.T_base_cam is None) or
                        (marker_id == self.next_block_id)
                    )

                    if not should_process:
                        continue
                    
                    if marker_id == 0:
                        size = marker_size_calibration
                    else:
                        size = marker_size_block

                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        [corners[idx]],
                        size,
                        camera_matrix,
                        dist_coeffs
                    )

                    rvec = rvecs[0]
                    tvec = tvecs[0]

                    T_cam_marker = rvec_tvec_to_T(rvec, tvec)

                    if marker_id == 0 and self.T_base_cam is None:
                        self.T_base_cam = T_base_marker @ np.linalg.inv(T_cam_marker)

                    if self.T_base_cam is not None:
                        T_base_block = self.T_base_cam @ T_cam_marker

                        if marker_id != 0:
                            block_poses_in_base[marker_id] = T_base_block

                        #cv2.drawFrameAxes(
                        #   frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03
                        #)

            # ----- Robot response queue -----
            try:
                while True:
                    resp = self.resp_queue.get_nowait()
                    if resp == "picked_and_placed":
                        self.next_block_id += 1
                        self.send_command = True
            except:
                pass

            # ----- Keyboard -----
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.cmd_queue.put(("shutdown", None))
                break

            if self.next_block_id in block_poses_in_base and self.send_command:
                self.send_command = False
                T_base_block = block_poses_in_base[self.next_block_id]
                self.cmd_queue.put(("pick_and_place", (self.next_block_id, T_base_block)))
                print(f"Sent pick_and_place request for block {self.next_block_id}")
            else:
                print("Block not visible!")

            if self.next_block_id == 7:
                break

            cv2.imshow("ArUco Block Detection + Robot Mapping", frame)

        try:
            self.cap.release()
        except:
            pass
        cv2.destroyAllWindows()
        print("VideoWorker stopped.")