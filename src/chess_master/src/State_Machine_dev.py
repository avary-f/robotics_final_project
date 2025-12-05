## This file is meant to test Baxter movement

## First task will be to just go to a point in space

## Second task will be to let us move the arm and get the q using ik 

#system level imports
import sys, os
sys.path.append(os.path.expanduser('~/Desktop/robotics_ws/src/rad_baxter_limb/src'))
from collections import deque
import numpy as np
import scipy.io as sio
#!/usr/local/bin/python

from copy import deepcopy
from threading import RLock, Timer
import time
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb.rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from enum import Enum
from trac_ik_python.trac_ik import IK

CALIB_FILE = os.path.expanduser("~/baxter_calibration.npz")

def save_calibration(calibration, desired_quat):
    np.savez(
        CALIB_FILE,
        angular=calibration.angular,
        cartesian=calibration.cartesian,
        quat=np.array(desired_quat)
    )
    print(f"Calibration saved to {CALIB_FILE}")


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        return None, None

    data = np.load(CALIB_FILE)
    calibration = CalibrationData(
        angular=data["angular"],
        cartesian=data["cartesian"]
    )
    desired_quat = data["quat"].tolist()
    print(f"Loaded calibration from {CALIB_FILE}")
    return calibration, desired_quat



# ------------------- CONFIGURABLE PARAMETERS -------------------
GRIPPER_CLOSED = 40   # 0 = fully closed, adjust as needed
GRIPPER_OPEN = 100    # 100 = fully open
# VERTICAL_OFFSET = 0.3048  # 1 foot in meters
VERTICAL_OFFSET = .05
A0_Y_OFFSET = -0.127      # 5 inches below board in meters
GRIPPER_OFFSET = np.array([0.0, 0.0, -0.15])   # same as Movement_test.py


class CalibrationData:
    def __init__(self, angular=None, cartesian=None):
        self.angular = angular
        self.cartesian = cartesian

class State(Enum):
    CALIBRATE = "CALIBRATE"
    NEUTRAL = "NEUTRAL"
    START_MOVEMENT = 'START_MOVEMENT'
    GRAB_PIECE = "GRAB_PIECE"
    PICK_UP = "PICK_UP"
    NEXT_MOVE = "NEXT_MOVE"
    PUT_DOWN = "PUT_DOWN"
    RESET = "RESET"



def create_gripper_client(side="right"):
    """
    Creates and returns an action client for Baxter's gripper.
    side: 'right' or 'left'
    """
    ns = f"/robot/end_effector/{side}_gripper/gripper_action"
    client = actionlib.SimpleActionClient(ns, GripperCommandAction)

    rospy.loginfo(f"Waiting for {side} gripper action server...")
    client.wait_for_server()
    rospy.loginfo(f"Connected to {side} gripper action server.")

    return client

def grip(client, position, effort=-1.0):
    """
    Send a goal to the gripper.
    position: 0 = closed, 100 = open
    effort: -1 uses gripper defaults
    """
    goal = GripperCommandGoal()
    goal.command.position = float(position)
    goal.command.max_effort = float(effort)

    client.send_goal(goal)
    rospy.loginfo(f"Gripper command sent. Position={position} effort={effort}")

def calibrate_corners(num_trials, limb):
    print("Move Baxter to the 4 corners of the chess board")
    print("Position 1: Bottom left corner")
    print("Position 2: Top left corner")
    print("Position 3: Bottom right corner")
    print("Position 4: Top right corner")

    angular_calibration = np.zeros((7, num_trials))
    cartesian_calibration = np.zeros((3, num_trials))

    for i in range(num_trials):
        input(f"\n\nGo to position {i+1} and then press Enter...")
        pose = limb.get_kdl_forward_position_kinematics()
        R = tf.transformations.quaternion_matrix(pose[3:])[0:3, 0:3]
        position = np.array(pose[0:3])
        q_cur = np.array(limb.get_joint_angles())

        print("\nJoint angles:\n", q_cur)
        print("\nEnd effector position:\n", position)
        print("\nEnd effector orientation:\n", R, "\n\n")

        angular_calibration[:, i] = q_cur
        cartesian_calibration[:, i] = position

    print("Angular calibration matrix:\n", angular_calibration)
    print("Cartesian calibration matrix:\n", cartesian_calibration)

    # Return as CalibrationData object
    return CalibrationData(angular_calibration, cartesian_calibration)

# def calculate_position(position, limb, calibration, tip_orientation, vertical_offset=VERTICAL_OFFSET):
#     num_squares_x = 8
#     num_squares_y = 8

#     # Extract 3D corner vectors from the calibration data
#     bl = calibration.cartesian[:, 0]   # bottom-left
#     tl = calibration.cartesian[:, 1]   # top-left
#     br = calibration.cartesian[:, 2]   # bottom-right
#     tr = calibration.cartesian[:, 3]   # top-right

#     # Compute board vectors
#     vect_x = (br - bl + tr - tl) / 2.0  # average x-axis direction (left→right)
#     vect_y = (tl - bl + tr - br) / 2.0  # average y-axis direction (bottom→top)

#     board_width = np.linalg.norm(vect_x)
#     board_height = np.linalg.norm(vect_y)

#     # Special case: "A0" goes above center of the board, 5 inches below board surface
#     if isinstance(position, str) and position.upper() == "A0":
#         target = bl + 0.5 * vect_x + (A0_Y_OFFSET / np.linalg.norm(vect_y)) * vect_y
#         target[2] += vertical_offset                   # apply vertical offset
#     else:
#         # --- Chess-style coordinate input ---
#         col_letter, row_num = position
#         col_idx = ord(col_letter.upper()) - ord('A')
#         if not (0 <= col_idx < num_squares_x):
#             raise ValueError(f"Invalid column '{col_letter}'. Must be A–H.")
#         row_idx = row_num - 1
#         if not (0 <= row_idx < num_squares_y):
#             raise ValueError(f"Invalid row '{row_num}'. Must be 1–8.")

#         # Compute target position on board
#         target = (
#             bl
#             + vect_x * (col_idx / num_squares_x)
#             + vect_y * (row_idx / num_squares_y)
#         )
#         target[2] += vertical_offset  # add standard vertical offset

#     # Calculate inverse kinematics
#     q_des = limb.kin_kdl.inverse_kinematics(target, tip_orientation)

#     return q_des, target

def calculate_position(position, limb, calibration, tip_orientation):
    num_squares_x = 8
    num_squares_y = 8

    # Extract calibrated corner positions
    bl = calibration.cartesian[:, 0]   # bottom-left
    tl = calibration.cartesian[:, 1]   # top-left
    br = calibration.cartesian[:, 2]   # bottom-right
    tr = calibration.cartesian[:, 3]   # top-right

    # Compute board axes
    vect_x = (br - bl + tr - tl) / 2.0
    vect_y = (tl - bl + tr - br) / 2.0

    # --- Chess-style coordinate input (A1, H8, etc.) ---
    col_letter, row_num = position

    col_idx = ord(col_letter.upper()) - ord('A')
    row_idx = row_num - 1

    # Compute ground-truth target (tip/gripper position)
    target = (
        bl
        + vect_x * (col_idx / num_squares_x)
        + vect_y * (row_idx / num_squares_y)
    )

    # Compute wrist target (move back by gripper offset)
    target_with_offset = target - GRIPPER_OFFSET

    # Seed solution = current joint angles
    seed = [limb.joint_angle(j) for j in limb.joint_names()]

    # Compute IK
    q_des = ik_solver.get_ik(
        seed,
        target_with_offset[0], target_with_offset[1], target_with_offset[2],
        tip_orientation[0], tip_orientation[1], tip_orientation[2], tip_orientation[3]
    )

    return q_des, target




def get_user_chess_position():
    """
    Prompt the user to enter a chessboard square (e.g., 'A1', 'g7').
    Valid columns: A-H (case-insensitive)
    Valid rows: 1-8
    Returns:
        tuple: (column_letter, row_number) e.g., ('A', 1)
    """
    valid_columns = [chr(c) for c in range(ord('A'), ord('H') + 1)]
    valid_rows = [str(r) for r in range(1, 9)]

    while True:
        user_input = input("Enter the desired chessboard position (e.g., A1, h7): ").strip()
        if len(user_input) < 2:
            print("Invalid input format. Try again.")
            continue

        col = user_input[0].upper()
        row = user_input[1:]

        if col not in valid_columns or row not in valid_rows:
            print("Invalid column or row. Columns: A-H, Rows: 1-8. Try again.")
            continue
        desired_move = (col, int(row))
        print(desired_move)
        return desired_move

def record_end_effector_orientation(limb):
    """
    Prompts the user to move the Baxter end effector to the desired orientation,
    then captures and returns the current orientation as a quaternion.
    
    Returns:
        np.array: Quaternion [x, y, z, w] of end effector
    """
    input("Manually move the end effector to the desired orientation and press Enter...")

    # Get current pose (position + orientation)
    current_pose = limb.get_kdl_forward_position_kinematics()
    
    # Orientation is the last 4 elements of the pose as quaternion
    current_quat = np.array(current_pose[3:])  # [x, y, z, w]

    print("Saved end effector orientation quaternion:", current_quat)
    tip_orientation = current_quat.flatten().tolist()

    # print(tip_orientation)
    # print("type: ", type(tip_orientation))
    # print("shape: ", np.shape(tip_orientation))

    return tip_orientation



def compute_q_for_target_with_offset(limb, target, tip_orientation, z_offset=0.05):
    """
    Compute joint angles so that the tip reaches target + vertical offset,
    taking GRIPPER_OFFSET into account.
    """
    target_offset = target.copy()
    target_offset[2] += z_offset
    target_with_gripper_offset = target_offset - GRIPPER_OFFSET
    seed = [limb.joint_angle(j) for j in limb.joint_names()]
    q_des_offset = ik_solver.get_ik(
        seed,
        target_with_gripper_offset[0],
        target_with_gripper_offset[1],
        target_with_gripper_offset[2],
        tip_orientation[0],
        tip_orientation[1],
        tip_orientation[2],
        tip_orientation[3]
    )
    return q_des_offset, target_offset

# def move_to_q_des(limb, q_des, target, speed=0.5, rate_hz=500, position_tol=0.001, timeout=130.0):
#     """
#     Move Baxter to the target joint angles q_des iteratively until the end-effector
#     reaches the target position within a specified tolerance or timeout is exceeded.
    
#     limb: RadBaxterLimb instance
#     q_des: target joint angles (7-element array)
#     target: target Cartesian position [x, y, z]
#     speed: maximum fraction of joint speed
#     rate_hz: control loop frequency
#     position_tol: tolerance in meters (1mm = 0.001 m)
#     timeout: maximum time to attempt movement (seconds)
#     """
#     limb.set_joint_position_speed(speed)
#     control_rate = rospy.Rate(rate_hz)

#     start_time = time.time()
#     print(f"Moving to target position: {target}")

#     while not rospy.is_shutdown():
#         # Check timeout
#         elapsed = time.time() - start_time
#         if elapsed > timeout:
#             print(f"Timeout reached ({timeout} sec). Stopping movement.")
#             break

#         # Command Baxter to target joint angles
#         limb.set_joint_positions_mod(q_des)

#         # Get current end-effector position
#         current_pose = limb.get_kdl_forward_position_kinematics()
#         current_pos = np.array(current_pose[0:3])

#         # Compute Euclidean distance to target
#         error = np.linalg.norm(current_pos - target)

#         if error <= position_tol:
#             print(f"Reached target! Error: {error*1000:.2f} mm")
#             break

#         control_rate.sleep()

def move_to_q_des(limb, q_des, target, speed=0.5, rate_hz=500, position_tol=0.07, timeout=10.0):
    """
    Move Baxter to the target joint angles q_des iteratively until the end-effector
    reaches the target position within a specified tolerance or timeout is exceeded.
    
    limb: RadBaxterLimb instance
    q_des: target joint angles (7-element array)
    target: target Cartesian position [x, y, z]
    speed: maximum fraction of joint speed
    rate_hz: control loop frequency
    position_tol: tolerance in meters (1mm = 0.001 m)
    timeout: maximum time to attempt movement (seconds)
    """
    limb.set_joint_position_speed(speed)
    control_rate = rospy.Rate(rate_hz)

    start_time = time.time()
    print(f"Moving to target position: {target}")

    print("q_des:", q_des)
    print("target:", target)

    while not rospy.is_shutdown():
        # Check timeout
        elapsed = time.time() - start_time
        if elapsed > timeout:
            print(f"Timeout reached ({timeout} sec). Stopping movement.")
            break

        # Command Baxter to target joint angles
        limb.set_joint_positions_mod(q_des)

        current_pose = limb.get_kdl_forward_position_kinematics()
        current_wrist = np.array(current_pose[0:3])

        # # Compute tip position from wrist + gripper offset
        # current_wrist = current_tip - GRIPPER_OFFSET

        # Compute Euclidean distance to target tip
        error = np.linalg.norm(current_wrist - target)
        print(f"Tip position error: {error:.4f} m")
        if error <= position_tol:
            print(f"Reached target! Error: {error*1000:.2f} mm")
            break

        control_rate.sleep()



def shutdown_robot(limb, gripper_client):
    """
    Safe shutdown procedure for Baxter.
    """
    rospy.loginfo("Shutting down Baxter safely...")

    # Stop all limb movements
    if limb is not None:
        try:
            limb.stop_all_motors()
            rospy.loginfo("Stopped all limb motors.")
        except Exception as e:
            rospy.logwarn(f"Failed to stop limb motors: {e}")

    # Open gripper
    if gripper_client is not None:
        try:
            grip(gripper_client, GRIPPER_OPEN)
            rospy.sleep(1)
            rospy.loginfo("Gripper opened for shutdown.")
        except Exception as e:
            rospy.logwarn(f"Failed to open gripper: {e}")

    rospy.loginfo("Shutdown complete.")


# def main(args=None):
#     rospy.init_node('me_537_lab')

#     # Initialize robot interfaces
#     limb = RadBaxterLimb('right')
#     gripper_client = create_gripper_client("right")
    

#     # Register shutdown function
#     rospy.on_shutdown(lambda: shutdown_robot(limb, gripper_client))

#     # Initialize state machine
#     current_state = State.CALIBRATE

#     calibration = None
#     desired_quat = None

#     try:
#         while not rospy.is_shutdown():
#             if current_state == State.CALIBRATE:
#                 rospy.loginfo("Calibrating board corners...")
#                 calibration = calibrate_corners(4, limb)
#                 desired_quat = record_end_effector_orientation(limb)
#                 current_state = State.START_MOVEMENT

#             elif current_state == State.NEUTRAL:
#                 rospy.loginfo("Moving to initial board position...")
#                 desired_chess_location = "A0"
#                 q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

#                 grip(gripper_client, GRIPPER_OPEN)
#                 rospy.sleep(1)

#                 move_to_q_des(limb, q_des, target, speed=0.5)
#                 current_state = State.START_MOVEMENT

#             elif current_state == State.START_MOVEMENT:
#                 rospy.loginfo("Select current piece position...")
#                 desired_chess_location = get_user_chess_position()

#                 # Get joint angles and original target from calculate_position
#                 q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

#                 q_des_offset, target_offset = compute_q_for_target_with_offset(limb, target, desired_quat, z_offset=0.05)
#                 move_to_q_des(limb, q_des_offset, target_offset, speed=0.25)

#                 # Move the arm using the same joint angles (wrist is already positioned correctly)
#                 move_to_q_des(limb, q_des, target_offset, speed=0.25)

#                 current_state = State.GRAB_PIECE

#             elif current_state == State.GRAB_PIECE:
#                 rospy.loginfo("Grabbing piece...")
#                 move_to_q_des(limb, q_des, target, speed=0.25)
#                 grip(gripper_client, GRIPPER_CLOSED)
#                 rospy.sleep(1)
#                 current_state = State.PICK_UP

#             elif current_state == State.PICK_UP:
#                 rospy.loginfo("Lifting piece...")
#                 move_to_q_des(limb, q_des, target_offset, speed=0.25)
#                 # Optionally move up a bit; can add function to increment Z
#                 current_state = State.NEXT_MOVE

#             elif current_state == State.NEXT_MOVE:
#                 rospy.loginfo("Select target position for piece...")
#                 desired_chess_location = get_user_chess_position()
#                 q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

#                 # Apply additional vertical offset to the target tip position
#                 vertical_offset = 0.05
#                 target_offset = target.copy()           # don't modify original target
#                 target_offset[2] += vertical_offset     # raise tip by 0.05 m

#                 move_to_q_des(limb, q_des, target, speed=0.25)
#                 current_state = State.PUT_DOWN

#             elif current_state == State.PUT_DOWN:
#                 rospy.loginfo("Placing piece down...")
#                 move_to_q_des(limb, q_des, target, speed=0.25)
#                 grip(gripper_client, GRIPPER_OPEN)
#                 rospy.sleep(1)
#                 current_state = State.RESET

#             elif current_state == State.RESET:
#                 rospy.loginfo("Resetting state machine for next move...")
#                 move_to_q_des(limb, q_des, target_offset, speed=0.25)
#                 # current_state = State.NEUTRAL

#             rospy.sleep(0.1)

#     except rospy.ROSInterruptException:
#         # Ctrl+C pressed; handled by shutdown_robot
#         rospy.loginfo("ROSInterruptException caught. Exiting safely.")


def main(args=None):
    rospy.init_node('me_537_lab')

    # Initialize robot interfaces
    limb = RadBaxterLimb('right')
    gripper_client = create_gripper_client("right")
    
    # Register shutdown function
    rospy.on_shutdown(lambda: shutdown_robot(limb, gripper_client))

    # Initialize state machine
    current_state = State.CALIBRATE

    calibration = None
    desired_quat = None

    try:
        while not rospy.is_shutdown():
            if current_state == State.CALIBRATE:
                # Try loading previous calibration
                calibration, desired_quat = load_calibration()

                if calibration is not None:
                    print("\nPrevious calibration found.")
                    use_old = input("Use stored calibration? (y/n): ").strip().lower()

                    if use_old == "y":
                        current_state = State.START_MOVEMENT
                        continue
                    else:
                        print("Re-calibrating...")
                
                # --- Perform new calibration ---
                calibration = calibrate_corners(4, limb)
                desired_quat = record_end_effector_orientation(limb)
                save_calibration(calibration, desired_quat)

                current_state = State.START_MOVEMENT


            elif current_state == State.NEUTRAL:
                rospy.loginfo("Moving to initial board position...")
                desired_chess_location = "A0"
                q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

                grip(gripper_client, GRIPPER_OPEN)
                rospy.sleep(1)

                move_to_q_des(limb, q_des, target, speed=0.5)
                current_state = State.START_MOVEMENT

            elif current_state == State.START_MOVEMENT:
                rospy.loginfo("Select current piece position...")
                desired_chess_location = get_user_chess_position()
                q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

                # Compute IK for tip raised 0.05 m
                q_des_offset, target_offset = compute_q_for_target_with_offset(limb, target, desired_quat, z_offset=0.15)
                move_to_q_des(limb, q_des_offset, target_offset, speed=0.25)

                current_state = State.GRAB_PIECE

            elif current_state == State.GRAB_PIECE:
                rospy.loginfo("Grabbing piece...")
                move_to_q_des(limb, q_des, target, position_tol=.02, speed=0.25)
                rospy.sleep(1)
                grip(gripper_client, GRIPPER_CLOSED)
                rospy.sleep(1)
                current_state = State.PICK_UP

            elif current_state == State.PICK_UP:
                rospy.loginfo("Lifting piece...")
                # Raise tip 0.05 m for pickup
                q_des_offset, target_offset = compute_q_for_target_with_offset(limb, target, desired_quat, z_offset=0.15)
                move_to_q_des(limb, q_des_offset, target_offset, speed=0.25)

                current_state = State.NEXT_MOVE

            elif current_state == State.NEXT_MOVE:
                rospy.loginfo("Select target position for piece...")
                desired_chess_location = get_user_chess_position()
                q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)

                # Raise tip 0.05 m above target
                q_des_offset, target_offset = compute_q_for_target_with_offset(limb, target, desired_quat, z_offset=0.15)
                move_to_q_des(limb, q_des_offset, target_offset, speed=0.25)

                current_state = State.PUT_DOWN

            elif current_state == State.PUT_DOWN:
                rospy.loginfo("Placing piece down...")
                move_to_q_des(limb, q_des, target, position_tol=.02, speed=0.25)
                rospy.sleep(1)
                grip(gripper_client, GRIPPER_OPEN)
                rospy.sleep(1)
                current_state = State.RESET

            elif current_state == State.RESET:
                rospy.loginfo("Resetting state machine for next move...")
                # Move back to raised neutral position
                q_des_offset, target_offset = compute_q_for_target_with_offset(limb, target, desired_quat, z_offset=0.15)
                move_to_q_des(limb, q_des_offset, target_offset, speed=0.25)
                current_state = State.START_MOVEMENT

                

            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught. Exiting safely.")




if __name__ == '__main__':
    ik_solver = IK(
        "base",
        "right_hand",
        timeout=0.15,
        epsilon=1e-5
    )
    main()
