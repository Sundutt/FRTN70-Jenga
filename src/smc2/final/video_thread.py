# video_thread.py
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import threading
import mediapipe as mp
from shared_state import pause_event
from queue import Empty

# Keep identical constants/params as original script
url = "http://klasthorgren:video123@10.29.147.128:8081/video" #Klas
url = "http://10.89.236.92:8080/video" #Fredriks
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

marker_size_calibration = 0.0748
marker_size_block = 0.0225

base_marker_pos = np.array([0.400, -0.300, 0.000])  # example
base_marker_rot = np.array([[-1, 0, 0],
                            [0, -1, 0],
                            [0, 0, 1]])

T_base_marker = np.eye(4)
T_base_marker[:3, :3] = base_marker_rot
T_base_marker[:3, 3] = base_marker_pos

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
        self.pick_command = False
        self.place_command = False
        self.first_block = True
        self.T_position_tower = []


        # ---- Faster Mediapipe config ----
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            model_complexity=0
        )

        # initialize capture (low latency settings)
        self.cap = cv2.VideoCapture(url) #cv2.CAP_FFMPEG)
        #self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        #self.cap.set(cv2.CAP_PROP_FPS, 10)
        #self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        #self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

        time.sleep(0.2)


    def stop(self):
        self._running = False

    def run(self):
        global aruco_dict, parameters

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                print("VT: Failed to grab frame")
                break

            self.detect_hand(frame)
            
            if self.pick_command or self.T_base_cam is None:
                self.pickup_new_block(frame)

            if self.place_command:
                self.place_block_in_tower(frame)

            # ----- Robot response queue -----
            try:
                resp, payload = self.resp_queue.get_nowait()
                if resp == "init":
                    print("VT: Got init from robot.")
                    self.T_position_tower = payload.copy()
                if resp == "picked_block":
                    print("VT: Will try to find pos to place block")
                    self.place_command = True
                if resp == "placed_block":
                    print("VT: Will try to find block to pick")
                    self.pick_command = True
            except Empty:
                pass

            # ----- Keyboard -----
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.cmd_queue.put(("shutdown", None))
                break

            cv2.imshow("ArUco Block Detection + Robot Mapping", frame)

        try:
            self.cap.release()
        except:
            pass
        cv2.destroyAllWindows()
        print("VT: VideoWorker stopped.")

    def detect_hand(self, frame):
        # ----- Hand detection (every frame, lightweight model) -----
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        hand_result = self.mp_hands.process(rgb)

        if hand_result.multi_hand_landmarks:
            #print("VT: HAND DETECTED!")
            pause_event.set()
            cv2.putText(
                frame,
                "Hand detected!",
                (300, 200),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (0, 0, 255),
                4
            )
            for hand_landmarks in hand_result.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                    mp.solutions.drawing_styles.get_default_hand_connections_style()
                )        

                    
        else:
            pause_event.clear()

    def pickup_new_block(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Use last known good markers for display and pose
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for idx, marker_id in enumerate(ids.flatten()):
                #print("VT: Pick up new block function currently sees. Marker_id = ", marker_id)

                # Decide if this marker should be processed
                if marker_id == 0 and self.T_base_cam is None:
                    size = marker_size_calibration
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        [corners[idx]],
                        size,
                        camera_matrix,
                        dist_coeffs
                    )

                    rvec = rvecs[0]
                    tvec = tvecs[0]
                    T_cam_marker = rvec_tvec_to_T(rvec, tvec)
                    if T_cam_marker[2, 2] < 0.95:
                        return

                    self.T_base_cam = T_base_marker @ np.linalg.inv(T_cam_marker)
                    
                    print("VT: Is calibrated now.")
                    self.pick_command = True

                if marker_id != 0 and self.T_base_cam is not None:
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

                    T_base_block = self.T_base_cam @ T_cam_marker

                    if T_base_block[2, 2] < 0.95:
                        #print("VT: Block: ", marker_id, " bad image")
                        continue
                    
                    if T_base_block[1, 3] > -0.3:
                        #print("VT: Block: ", marker_id, " restricted zone, y = ", T_base_block[1, 3])

                        continue                     
                    
                    # Send pick command
                    print("VT: Found new block ", marker_id, " to pick up")
                    self.cmd_queue.put(("pick_block", T_base_block))
                    self.pick_command = False
                    break

    def place_block_in_tower(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        height = 0
        low_layer = True

        blocks_in_tower = []

        placing_position = 0

        if self.first_block:
            print("VT: First block, placing on pos 0")
            self.first_block = False
            self.cmd_queue.put(("place_block", (placing_position, height)))
            self.place_command = False
            return
            
        if ids is not None: 
            aruco.drawDetectedMarkers(frame, corners, ids)
            for idx, marker_id in enumerate(ids.flatten()):
                rvec, tvec = None, None

                # If a size is wrong, pose estimation breaks, so this is important
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    [corners[idx]],
                    marker_size_block,
                    camera_matrix,
                    dist_coeffs
                )

                rvec = rvecs[0]
                tvec = tvecs[0]

                T_cam_marker = rvec_tvec_to_T(rvec, tvec)

                if self.T_base_cam is None:
                    print("VT: ERROR: T_base_cam not initialized yet")
                    continue

                T_base_block = self.T_base_cam @ T_cam_marker
                 
                if T_base_block[1, 3] < -0.3 or marker_id == 0:
                    #print("VT: Place block function. Block: ", marker_id, " not on Tower")
                    continue
                if T_base_block[2,2] < 0.95:
                    return 
                for i in range(6):
                    if (self.T_position_tower[i][0,3]-0.015 <= T_base_block[0, 3] <= self.T_position_tower[i][0,3]+0.015) and (self.T_position_tower[i][1,3]-0.015 <= T_base_block[1, 3] <= self.T_position_tower[i][1, 3]+0.015):
                        print("VT: Place block function. Block: ", marker_id, " on tower position: ", i, " . x = ", T_base_block[0, 3], " y= ", T_base_block[1,3])
                        #T_base_block in pos i
                        if i == 1 or i == 4:
                            if -0.5 <= T_base_block[0,0] <= 0.5:
                                blocks_in_tower.append(4)
                                if T_base_block[2, 3] > height:
                                    height = T_base_block[2, 3]
                                    low_layer = False
                                break
                            else:
                                blocks_in_tower.append(1)
                                if T_base_block[2, 3] > height:
                                    height = T_base_block[2, 3]
                                    low_layer = True
                                break
                        blocks_in_tower.append(i)
                        if T_base_block[2, 3] > height:
                            height = T_base_block[2, 3]
                            if i < 3:
                                low_layer = True
                            else:
                                low_layer = False  
                        break
                 
            #decide where to place the next block
            if low_layer:
                for i in range(3):
                    if i not in blocks_in_tower:
                        placing_position = i
                        break
                else:
                    if not blocks_in_tower:
                        placing_position = 0
                    else:
                        placing_position = 3

            else:
                for i in range(3, 6):
                    if i not in blocks_in_tower:
                        placing_position = i
                        break
                else:
                    placing_position = 0
            print("VT: Place block function. Placing block on position: ", placing_position) 
            self.cmd_queue.put(("place_block", (placing_position, height)))
            self.place_command = False
