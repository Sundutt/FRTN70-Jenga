from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL

import argparse
import numpy as np
import pinocchio as pin
import time

nbr_layers = 3
block_width = 0.026
block_length = 0.07
block_height = 0.015
tool_offset = 0.145

def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    #parser.set_defaults()
    parser.description = "Build Jenga tower with fixed poses."
    parser = getClikArgs(parser)
    return parser.parse_args()

def pick_up_new_block():
        # Go to upper position
        T_w_goal.translation = np.array([0.4, -0.4, 0.1+tool_offset])
        print(f"Move above pick up location: {T_w_goal.translation}")
        moveL(args, robot, T_w_goal)
        T_w_goal.rotation = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
        ])
        print("Rotating to picking rotation")
        moveL(args, robot, T_w_goal)

        # Pick up
        T_w_goal.translation = np.array([0.4, -0.4, tool_offset])
        print(f"Moving to pick up location: {T_w_goal.translation}")
        moveL(args, robot, T_w_goal)
        robot.sendVelocityCommandToReal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(1.0)
        robot.closeGripper()
        time.sleep(1.0)

        # Move up again
        T_w_goal.translation = np.array([0.4, -0.4, 0.1+tool_offset])
        print(f"Moving back up: {T_w_goal.translation}")
        moveL(args, robot, T_w_goal)

def place_block(current_block, current_layer):
    # Upper position above tower
    if current_layer % 2 == 0:  # even layer
        print("Even layer")
        T_w_goal.translation = np.array([0.1 + block_width - (current_block*block_width), -0.45 - block_width, 0.3+tool_offset])
        print(f"Moving above tower: {T_w_goal.translation}")
        moveL(args, robot, T_w_goal)
        T_w_goal.rotation = np.array([
        [0, 1,  0],
        [1, 0,  0],
        [0, 0, -1]
        ])
        print("Rotating to even rotation")
        moveL(args, robot, T_w_goal)
    else:  # odd layer
        print("Odd layer")
        T_w_goal.translation = np.array([0.1, -0.45 - (current_block*block_width), 0.3+tool_offset])
        print(f"Moving above tower: {T_w_goal.translation}")
        moveL(args, robot, T_w_goal)
        T_w_goal.rotation = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1]
        ])
        print("Rotating to odd rotation")
        moveL(args, robot, T_w_goal)
    
    # Lower block and realease
    T_w_goal.translation[2] = (current_layer*block_height) - block_height + tool_offset
    print(f"Moving to place location: {T_w_goal.translation}")
    moveL(args, robot, T_w_goal)
    robot.sendVelocityCommandToReal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot.openGripper()
    time.sleep(1.0)

    # Move up
    T_w_goal.translation[2] = 0.3+tool_offset
    print(f"Moving back up: {T_w_goal.translation}")
    moveL(args, robot, T_w_goal)


if __name__ == "__main__":
    args = get_args()
    robot = getRobotFromArgs(args)

    # Define goal pose (T_w_goal)
    # Translation in meters: x, y, z relative to robot base
    robot_rotation = np.array([[1, 0, 0],
                                [0, -1, 0],
                                [0, 0, -1]])
    robot_position = np.array([0.3, -0.3, 0.3])
    T_w_goal = pin.SE3(robot_rotation, robot_position)
    print(f"Moving to home position: {robot_position}")
    moveL(args, robot, T_w_goal)
    robot.openGripper()
    
    try:?!?jedi=0, ?!?              (start: int, *_*stop: int*_*, step: int=...) ?!?jedi?!?
        #Build Jenga ?!?jedi=0, tover?!? (stop: int) ?!?jedi?!?
        for i in range(1, nbr_layers+1):
            for k in range(0, 3):
               pick_up_new_block()
               place_block(k, i) 

    except KeyboardInterrupt:
        print("Interrupted by user.")
    
    # Cleanup
    if args.visualizer:
        robot.killManipulatorVisualizer()

    if args.save_log:
        robot._log_manager.saveLog()

    if args.real:
        robot.stopRobot()
