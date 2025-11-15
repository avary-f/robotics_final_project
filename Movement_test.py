## This file is meant to test Baxter movement


## To try the new IK function, intall TRAC-IK 
# sudo apt install ros-<distro>-trac-ik
# pip install trac-ik-python



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
import trac_ik_python.trac_ik as trac_ik


class CalibrationData:
    def __init__(self, angular=None, cartesian=None):
        self.angular = angular
        self.cartesian = cartesian



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



def calculate_position(position, limb, calibration, tip_orientation, ik_solver, nominal):
    num_squares_x = 8
    num_squares_y = 8

    # Extract 3D corner vectors from the calibration data
    bl = calibration.cartesian[:, 0]   # bottom-left
    tl = calibration.cartesian[:, 1]   # top-left
    br = calibration.cartesian[:, 2]   # bottom-right
    tr = calibration.cartesian[:, 3]   # top-right

    # Compute board vectors
    vect_x = (br - bl + tr - tl) / 2.0  # average x-axis direction (left→right)
    vect_y = (tl - bl + tr - br) / 2.0  # average y-axis direction (bottom→top)

    board_width = np.linalg.norm(vect_x)
    board_height = np.linalg.norm(vect_y)

    square_width = board_width / num_squares_x
    square_height = board_height / num_squares_y

    # --- Chess-style coordinate input ---
    # Expect something like ('A', 1) or ('G', 7)
    col_letter, row_num = position

    # Convert 'A'–'H' to 0–7
    col_idx = ord(col_letter.upper()) - ord('A')
    if not (0 <= col_idx < num_squares_x):
        raise ValueError(f"Invalid column '{col_letter}'. Must be A–H.")

    # Convert 1–8 to 0–7 (bottom→top)
    row_idx = row_num - 1
    if not (0 <= row_idx < num_squares_y):
        raise ValueError(f"Invalid row '{row_num}'. Must be 1–8.")

    # Compute target position
    target = (
        bl
        + vect_x * (col_idx / num_squares_x)
        + vect_y * (row_idx / num_squares_y)
    )



    # calculate the calculated q_des 
    # q_des = limb.kin_kdl.inverse_kinematics(target, tip_orientation)

    q_des_list = ik_solver.get_ik(nominal.tolist(), target.tolist(), tip_orientation)

    if q_des_list is None:
        raise RuntimeError("TRAC-IK could not find a solution for the target")
    q_des = np.array(q_des_list)

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



def move_to_q_des(limb, q_des, target, speed=0.5, rate_hz=500, position_tol=0.001, timeout=60.0):
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

    while not rospy.is_shutdown():
        # Check timeout
        elapsed = time.time() - start_time
        if elapsed > timeout:
            print(f"Timeout reached ({timeout} sec). Stopping movement.")
            break

        # Command Baxter to target joint angles
        limb.set_joint_positions_mod(q_des)

        # Get current end-effector position
        current_pose = limb.get_kdl_forward_position_kinematics()
        current_pos = np.array(current_pose[0:3])

        # Compute Euclidean distance to target
        error = np.linalg.norm(current_pos - target)

        if error <= position_tol:
            print(f"Reached target! Error: {error*1000:.2f} mm")
            break

        control_rate.sleep()



if __name__ == '__main__':
    rospy.init_node('me_537_lab')
    limb = RadBaxterLimb('right')
    # Use the Baxter URDF or KDL chain if available; here is a simple example:
    base_link = 'base'       # Replace with Baxter base link name
    tip_link = 'right_gripper'  # Replace with Baxter tip link
    urdf_string = limb.get_urdf()  # Or load from file if available

    # Joint limits (optional, can be read from Baxter URDF)
    lower_limits = [-3.05, -1.70, -3.05, -1.70, -3.05, -1.70, -3.05]
    upper_limits = [3.05, 1.70, 3.05, 1.70, 3.05, 1.70, 3.05]
    nominal = np.zeros(7)

    ik_solver = trac_ik.IK(base_link, tip_link, urdf_string,
                        lower_limits, upper_limits, nominal)



    calibration = calibrate_corners(4, limb)
    desired_quat = record_end_effector_orientation(limb)
    desired_chess_location = get_user_chess_position()
    # q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat)
    q_des, target = calculate_position(desired_chess_location, limb, calibration, desired_quat, ik_solver, nominal)
    move_to_q_des(limb, q_des, target, speed=.25)

    