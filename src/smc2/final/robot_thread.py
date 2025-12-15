import rtde_control
import rtde_receive
from scipy.spatial.transform import Rotation as R_scipy
import xmlrpc.client
import numpy as np
import time
from shared_state import pause_event

# Robot / gripper IPs (same values as before)
ROBOT_IP = "192.168.1.150"
GRIPPER_IP = "192.168.1.1"

# Tower and geometry constants (identical to original)
block_width = 0.03
block_height = 0.015

tower_origin_base = np.array([0.3, -0.25, 0.3])

even_rotation = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
odd_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

T_position_tower = []
for block in range(6):
    
    T_pos = np.eye(4)
    T_pos[:3, 3] = tower_origin_base.copy() + np.array([
        block_width*(1-block%3)*(block//3==0), 
        block_width*(1-block%3)*(block//3==1), 
        -0.3])
    T_pos[:3, :3] = even_rotation if (block//3==0) else odd_rotation
    T_position_tower.append(T_pos)


# Initialize RTDE interfaces and gripper
class RobotWorker:
    def __init__(self, cmd_queue, resp_queue):
        self.cmd_queue = cmd_queue
        self.resp_queue = resp_queue
        # initialize robot interfaces
        print("Initializing RTDEControlInterface and RTDEReceiveInterface...")
        self.rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        self.cb = xmlrpc.client.ServerProxy(f"http://{GRIPPER_IP}:41414/")
        print("RTDE interfaces initialized.")

        self.init = True

    def open_gripper(self):
        """Fully open the 2FG7 using maximum external width."""
        time.sleep(0.5)
        max_width = self.cb.twofg_get_max_external_width(0)
        self.cb.twofg_grip_external(0, max_width, 140, 50)   # (id, width, force, speed)
        time.sleep(0.5)

    def close_gripper(self):
        """Fully close the 2FG7."""
        self.cb.twofg_grip_external(0, 0.0, 140, 50)
        time.sleep(0.5)

    def moveL(self, t, R, speed=0.2, accel=0.4):
        """
        Move UR robot linearly (moveL) with translation vector t and rotation matrix R.
        """
        rot = R_scipy.from_matrix(R)
        rx, ry, rz = rot.as_rotvec()

        t = np.array(t).reshape(3,)
        pose = [t[0], t[1], t[2], rx, ry, rz]
        # Start async motion
        self.rtde_c.moveL(pose, speed, accel, True)

        last_state = None

        while True:
            # Pause if hand detected
            if pause_event.is_set():
                self.rtde_c.stopL(0.7)
                # Wait until hand disappears
                while pause_event.is_set():
                    time.sleep(0.05)
                #time.sleep(0.5)
                # Resume movement
                self.rtde_c.moveL(pose, speed, accel, True)

            # Poll async motion state
            state = self.rtde_c.getAsyncOperationProgress()

            # Motion finished when it toggles negative
            if state < 0 and state != last_state:
                break

            last_state = state
            time.sleep(0.1)

    def pick_up_block_from_vision(self, T_base_block):
        self.open_gripper()
        # Approach from above
        pickup_height_offset = 0.02  # approach 10 cm above detected block
        t_above = T_base_block[:3, 3].copy()
        t_above[2] += pickup_height_offset

        R_block = T_base_block[:3, :3]
        R_flip = R_scipy.from_euler('x', np.pi).as_matrix()      # Flip down
        R_z90 = R_scipy.from_euler('z', np.pi/2).as_matrix()     # Rotate around Z
        R_gripper = R_block @ R_flip @ R_z90

        # Move above block
        self.moveL(t_above, R_gripper)

        #t_pickup = t_above.copy()
        #t_pickup[2] -= 0.118
        self.rtde_c.moveUntilContact([0, 0, -0.02, 0, 0, 0])

        #moveL(t_pickup, R_gripper)

        self.close_gripper()

        # Move back up
        self.moveL(t_above, R_gripper)
        self.moveL(tower_origin_base, R_gripper)

    def place_block(self, pos, height, clamping):
        placing_height_offset = 0.02
        T_place = T_position_tower[pos].copy()
        T_place[2, 3] = height + placing_height_offset

        self.moveL(T_place[:3, 3], T_place[:3, :3])

        self.rtde_c.moveUntilContact([0, 0, -0.02, 0, 0, 0])

        self.open_gripper()

        if clamping:
            new_pose = T_place.copy()

            # get z value
            new_pose[2, 3] = self.rtde_r.getActualTCPPose()[2]

            # higher 4 cm
            T_place[2, 3] = new_pose[2, 3] + 0.04
            self.moveL(T_place[:3, 3], T_place[:3, :3])

            # move to x-y orgin and turn gripper
            T_place[:2, 3] = tower_origin_base[:2]
            if pos == 2 or pos == 1:
                T_place[:3, :3] = odd_rotation
            else:
                T_place[:3, :3] = even_rotation

            self.moveL(T_place[:3, 3], T_place[:3, :3])

            # lower again
            T_place[2, 3] = new_pose[2, 3]
            self.moveL(T_place[:3, 3], T_place[:3, :3])

            self.close_gripper()
            self.open_gripper()
        # raise end effector
        T_place[2, 3] += 0.20
        self.moveL(T_place[:3, 3], T_place[:3, :3])
                    
    def run(self):
        print("RobotWorker started, waiting for commands...")
        running = True
        while running:
            if self.init:
                self.resp_queue.put(("init", T_position_tower))
                self.init = False

            cmd, payload = self.cmd_queue.get()

            if cmd == "shutdown":
                print("RobotWorker received shutdown.")
                running = False
                break

            if cmd == "pick_block":
                T_base_block = payload
                try:
                    print("RT: Got cmd to pick up block on T=\n", T_base_block)
                    self.pick_up_block_from_vision(T_base_block)
                    try:
                        self.resp_queue.put(("picked_block", None))
                    except Exception:
                        pass
                except Exception as e:
                    print(f"Exception while performing pick_block: {e}")

            if cmd == "place_block":
                pos, height, clamping = payload
                try:
                    print("RT: got cmd to place block on T=\n", T_base_block)
                    self.place_block(pos, height, clamping)
                    try:
                        self.resp_queue.put(("placed_block", None))
                    except Exception:
                        pass
                except Exception as e:
                    print(f"Exception while performing place_block: {e}")

        print("RobotWorker stopped.")
