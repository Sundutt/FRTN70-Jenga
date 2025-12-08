import rtde_control
import rtde_receive
import numpy as np
import time
import cv2
from scipy.spatial.transform import Rotation as R

ROBOT_IP = "192.168.1.150"

rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

def moveL_interruptible(target_xyzrpy, speed=0.1, threshold=0.002):
    """
    Moves linearly toward target_xyzrpy but checks for interrupt key.
    Stops when close enough or stopped.
    """

    # OpenCV dummy window (for keypress detection)
    cv2.namedWindow("Control Window")

    while True:
        current_pose = np.array(rtde_r.getActualTCPPose())  # [x,y,z,rx,ry,rz]

        # Distance error in translation
        error_vec = target_xyzrpy[:3] - current_pose[:3]
        distance = np.linalg.norm(error_vec)

        if distance < threshold:
            print("Reached target.")
            rtde_c.speedStop()
            break

        if distance < 0.025:   # slowdown zone 2 cm from target
            speed = speed * (distance / 0.03)
            if speed < 0.002:   # minimum creeping speed
                speed = 0.002
        
        # Convert direction vector to velocity

        direction = error_vec / distance
        velocity = direction * speed
        vel6 = [velocity[0], velocity[1], velocity[2], 0, 0, 0]

        rtde_c.speedL(vel6, 0.25, 0.05)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("STOP requested by user!")
            rtde_c.speedStop()
            break
        
        if key == ord('q'):
            print("Quit requested")
            rtde_c.speedStop()
            break

        time.sleep(0.01)

# --------------------------
# Test Example
# --------------------------

# Current pose
start_pose = rtde_r.getActualTCPPose()

# Move 20cm forward in X — just for demo
target_pose = start_pose.copy()
target_pose[0] -= 0.10
target_pose[1] -= 0.10
target_pose[2] += 0.10


print("Press 's' to stop early or 'q' to quit.")
moveL_interruptible(target_pose)
robot_pose = rtde_r.getActualTCPPose()
print(f"Target pose: {target_pose} \n Robot pose: {robot_pose}.")

target_pose[0] -= 0.10
target_pose[1] -= 0.10
target_pose[2] += 0.10

#moveL_interruptible(target_pose)
robot_pose = rtde_r.getActualTCPPose()
print(f"Target pose: {target_pose} \n Robot pose: {robot_pose}.")

move_goal = rtde_r.getActualTCPPose()



rtde_c.stopScript()
cv2.destroyAllWindows()
