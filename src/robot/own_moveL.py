from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL, moveUntilContact

import argparse
import numpy as np
import pinocchio as pin
import time
import cv2
import cv2.aruco as aruco



def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    parser.set_defaults(
    robot_ip="192.168.1.150",
    plotter=False,
    gripper="onrobot",
    goal_error="0.002",
    contact_detecting_force="1.5",    
    )
    parser.description = "Own moveL."
    parser = getClikArgs(parser)
    return parser.parse_args()


def moveL_interupt(robot, target_T, speed=0.1, threshold=0.002):
    cv2.namedWindow("Control Window")

    while True:
        # Pinocchio SE3 objects from 4×4 matrices
        T_current = pin.SE3(robot.T_w_e)
        T_target  = pin.SE3(target_T)

        # Relative transform in end-effector frame
        T_err = T_current.inverse() * T_target

        # Log map → 6D motion vector (vx, vy, vz, wx, wy, wz)
        err_se3 = pin.log(T_err)
        trans_err = err_se3.linear     # translation error (3×1)
        rot_err   = err_se3.angular    # rotation error (3×1)

        distance = np.linalg.norm(trans_err)    

        if distance < threshold:
            print("Reached target.")
            robot.sendVelocityCommand(np.array([0, 0, 0, 0, 0, 0]))
            break

        if distance < 0.025:   # slowdown zone 2 cm from target
            speed = speed * (distance / 0.03)
            if speed < 0.002:   # minimum creeping speed
                speed = 0.002
        
        # Convert direction vector to velocity

        direction = trans_err / distance
        velocity = direction * speed
        vel6 = np.array([velocity[0], velocity[1], velocity[2], 0, 0, 0])

        robot.sendVelocityCommand(vel6)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("STOP requested by user!")
            robot.sendVelocityCommand(np.array([0, 0, 0, 0, 0, 0]))
            break
        
        if key == ord('q'):
            print("Quit requested")
            robot.sendVelocityCommand(np.array([0, 0, 0, 0, 0, 0]))
            break

        time.sleep(0.01)

if __name__ == "__main__":
    args = get_args()
    robot = getRobotFromArgs(args)
    robot._step()
    try:

        target_pose = robot.T_w_e

        # Move 20cm forward in X — just for demo
        target_pose.translation += np.array([-0.1, -0.1, 0.1])

        print("Press 's' to stop early or 'q' to quit.")
        moveL_interupt(robot, target_pose)
        robot_pose = robot.T_w_e
        print(f"Target pose: {target_pose} \n Robot pose: {robot_pose}.")

        #target_pose.translation += np.array([-0.1, -0.1, 0.1])
        
        #moveL_interruptible(target_pose)
        #robot_pose = robot.T_w_e
        #print(f"Target pose: {target_pose} \n Robot pose: {robot_pose}.")


    except KeyboardInterrupt:
        print("Interrupted by user.")
    
    # Cleanup
    if args.visualizer:
        robot.killManipulatorVisualizer()

    if args.save_log:
        robot._log_manager.saveLog()

    if args.real:
        robot.stopRobot()
    cv2.destroyAllWindows()
