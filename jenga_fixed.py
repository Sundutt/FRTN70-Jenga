from smc import getMinimalArgParser, getRobotFromArgs
from smc.control.cartesian_space import getClikArgs
from smc.control.cartesian_space.cartesian_space_point_to_point import moveL, moveUntilContact

import argparse
import numpy as np
import pinocchio as pin
import time

nbr_layers = 4

block_width = 0.026
block_length = 0.07
block_height = 0.015
tool_offset = 0.148 #0.145

even_rotation = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
odd_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    
def get_args() -> argparse.Namespace:
    parser = getMinimalArgParser()
    parser.set_defaults(
    robot_ip="192.168.1.150",
    plotter=False,
    visualizer=False,
    gripper="onrobot",
    goal_error="0.002",
    contact_detecting_force="1.5",    
    )
    parser.description = "Build Jenga tower with fixed poses."
    parser = getClikArgs(parser)
    return parser.parse_args()

# ---------- PRIMITIVE FUNCTIONS ----------
def move_wstop(robot_rotation, robot_position):
    T = pin.SE3(robot_rotation, robot_position)
    print(f"Moving to: {robot_position}")
    moveL(args, robot, T)
    robot.stopRobot()
    #robot.sendVelocityCommandToReal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

def move_nostop(robot_rotation, robot_position):
    T = pin.SE3(robot_rotation, robot_position)
    print(f"Moving to: {robot_position}")
    moveL(args, robot, T)

def movecontact_wstop(speed=np.array([0, 0, 0.02, 0, 0, 0])):
    print(f"Moving until contact with speed: {speed}")
    moveUntilContact(args, robot, speed)
    robot.stopRobot()
 
def close_wsleep(sleep_duration=1.0):
    robot.closeGripper()
    time.sleep(sleep_duration)

def open_wsleep(sleep_duration=1.0):
    robot.openGripper()
    time.sleep(sleep_duration)

# ---------- JENGA FUNCTIONS ----------
def pick_up_new_block(current_layer):
    # Go to upper position
    pickup_rotation = odd_rotation
    above_pickup_position = np.array([0.4, -0.4, block_height*(current_layer+1)+tool_offset])
    print("Move above pick up location.")
    move_wstop(pickup_rotation, above_pickup_position)

    # Pick up
    on_pickup_position = np.array([0.4, -0.4, tool_offset])
    print("Moving to pick up location.")
    #move_wstop(pickup_rotation, on_pickup_position)
    movecontact_wstop()
    close_wsleep()

    # Move up again
    print("Moving back up.")
    move_wstop(pickup_rotation, above_pickup_position)


def place_block(current_block, current_layer):
    # Upper position above tower
    is_odd_layer = False
    placing_rotation = np.zeros((3, 3))
    above_placing_position = np.zeros(3)
    on_placing_position = np.zeros(3)

    if current_layer % 2 == 0:  # even layer
        is_odd_layer = False
        print("Even layer")
        placing_rotation = even_rotation
        above_placing_position = np.array([0.25 + block_width*(1-current_block), -0.35-block_width, block_height*(current_layer+2)+tool_offset])
        on_placing_position = np.array([0.25+block_width*(1-current_block), -0.35-block_width, block_height*(current_layer-1)+tool_offset])
    else:  # odd layer
        is_odd_layer = True
        print("Odd layer")
        placing_rotation = odd_rotation
        above_placing_position = np.array([0.25, -0.35-(current_block*block_width), block_height*(current_layer+2)+tool_offset])
        on_placing_position = np.array([0.25, -0.35-(current_block*block_width), block_height*(current_layer-1)+tool_offset])
    
    print("Moving above tower.")
    move_wstop(placing_rotation, above_placing_position)
    
    # Lower block and realease
    print("Moving to place location.")
    movecontact_wstop()
    last_placed_position = robot.T_w_e.translation
    open_wsleep()
 
    if current_block == 2:   
        #Merge 
        merge_blocks(is_odd_layer, last_placed_position)        
    else:
        # Move up
        print("Moving back up.")
        move_wstop(placing_rotation, above_placing_position)

def merge_blocks(is_odd_layer, placing_position):
    if is_odd_layer:
        above_merge_position = placing_position + np.array([0, block_width, 2*block_height])
        on_merge_position = placing_position + np.array([0, block_width, 0.0*block_height])
        print("Moving above merge position.")
        move_wstop(odd_rotation, above_merge_position)
        move_wstop(even_rotation, above_merge_position)
        print("Moving to merge position.")
        move_wstop(even_rotation, on_merge_position)
        #movecontact_wstop()
        close_wsleep()
        open_wsleep()
        print("Moving back up.")
        move_wstop(even_rotation, above_merge_position)
    else:
        above_merge_position = placing_position + np.array([block_width, 0, 2*block_height])
        on_merge_position = placing_position + np.array([block_width, 0, 0.0*block_height])
        print("Moving above merge position.")
        move_wstop(even_rotation, above_merge_position)
        move_wstop(odd_rotation, above_merge_position)
        print("Moving to merge position.")
        move_wstop(odd_rotation, on_merge_position)
        #movecontact_wstop()
        close_wsleep()
        open_wsleep()
        print("Moving back up.")
        move_wstop(odd_rotation, above_merge_position)
    


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
    
    try:
        #Build Jenga tover
        for i in range(1, nbr_layers+1):
            for k in range(0, 3):
                pick_up_new_block(i)
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
