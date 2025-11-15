import numpy as np
# from baxter_pykdl import baxter_kinematics as b_kin

# VizScene imports
import kinematics as kin
from visualization import VizScene
import kinematics_baxter as kin_baxter


# Setting up the DH visualization 

dh = [[0, .27035, .069, -pi/2],
      [pi/2, 0, 0, pi/2],
      [0, .36435, .069, -pi/2],
      [0, 0, 0, pi/2],
      [0, .37429, .010, -pi/2],
      [0, 0, 0, pi/2],
      [0, .229525, 0, 0]
]

joint_types = ['r', 'r', 'r', 'r', 'r', 'r', 'r']
arm = kin.SerialArm(dh, jt=joint_types)





# -------------------------------
# Calibration + Target Setup
# -------------------------------
calibration = {
    "cartesian": np.array([
        [0.6, 0.1, 0.1],  # Bottom Left
        [0.6, 0.4, 0.1],  # Top Left
        [0.9, 0.1, 0.1],  # Bottom Right
        [0.9, 0.4, 0.1],  # Top Right
    ]),
    "angular": np.array([
        [0.5, -1.0, 0.7, 1.5, -0.3, 1.0, 0.2],
        [0.6, -0.9, 0.8, 1.4, -0.4, 1.1, 0.3],
        [0.4, -1.1, 0.6, 1.6, -0.2, 0.9, 0.1],
        [0.7, -0.8, 0.9, 1.3, -0.5, 1.2, 0.4],
    ])
}

tip_orientation = [0, 0, 0, 1]  # Example orientation quaternion

def calculate_target(position, calibration):
    num_squares_x = 8
    num_squares_y = 8
    bl, tl, br, tr = calibration["cartesian"]

    vect_x = (br - bl + tr - tl) / 2.0
    vect_y = (tl - bl + tr - br) / 2.0

    col_letter, row_num = position
    col_idx = ord(col_letter.upper()) - ord('A')
    row_idx = row_num - 1

    target = bl + vect_x * (col_idx / num_squares_x) + vect_y * (row_idx / num_squares_y)
    return target

# -------------------------------
# Main
# -------------------------------
if __name__ == "__main__":
    # Ask for square
    user_in = input("Enter desired square (e.g., A1, C5): ").strip()
    col = user_in[0].upper()
    row = int(user_in[1])
    target = calculate_target((col, row), calibration)

    # Solve IK
    # kin = b_kin("right")
    q_des = kin_baxter.inverse_kinematics(target, tip_orientation)

    if q_des is None:
        print("IK failed")
        exit(0)
    print("Target:", target)
    print("Joint angles:", np.round(q_des, 3))

    # -------------------------------
    # Visualize in VizScene
    # -------------------------------
    scene = Scene()

    # Load Baxter URDF (replace with your local path)
    baxter = URDFModel(scene, "BaxterRightArm", "path_to_baxter_right_arm.urdf")
    scene.add_model(baxter)

    # Set joint positions from IK
    joint_names = ["right_s0", "right_s1", "right_e0", "right_e1",
                   "right_w0", "right_w1", "right_w2"]

    for name, val in zip(joint_names, q_des):
        baxter.set_joint_position(name, val)

    # Render VizScene window
    scene.run()
