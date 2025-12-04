#!/usr/bin/env python3
import rospy
from trac_ik_python.trac_ik import IK
import numpy as np

rospy.init_node("baxter_trac_ik_test")

# Set up TRAC-IK for Baxter right arm
ik = IK(
    "base",         # root link
    "right_hand",   # tip link (use "right_gripper" if using gripper)
    timeout=0.015,
    epsilon=1e-5
)

# Example target position (meters)
pos = [0.7, -0.1, 0.15]      # x, y, z

# Example orientation (quaternion: x, y, z, w)
quat = [0, 1, 0, 0]          # pointing downward

# Seed state: use Baxter's neutral pose or zeros
seed = [0.0]*7

# Call TRAC-IK
q_sol = ik.get_ik(
    seed,
    pos[0], pos[1], pos[2],
    quat[0], quat[1], quat[2], quat[3]
)

print("IK Solution:", q_sol)
