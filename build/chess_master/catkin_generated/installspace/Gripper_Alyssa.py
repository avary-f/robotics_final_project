#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def send_goal(client, position, effort=-1.0):
    """Send a goal to the gripper action server."""
    goal = GripperCommandGoal()
    goal.command.position = position   # 0.0 = closed, 100.0 = open
    goal.command.max_effort = effort   # -1 uses default
    client.send_goal(goal)
    rospy.loginfo("Sent goal: position=%.1f effort=%.1f", position, effort)

def main():
    rospy.init_node("gripper_cycle_action_client")

    # Action server for RIGHT gripper
    action_ns = "/robot/end_effector/right_gripper/gripper_action"

    rospy.loginfo("Waiting for gripper action server...")
    client = actionlib.SimpleActionClient(action_ns, GripperCommandAction)
    client.wait_for_server()
    rospy.loginfo("Connected to gripper action server.")

    # ---- Close ----
    rospy.loginfo("Closing gripper...")
    send_goal(client, 0.0)   # 0 = closed
    rospy.sleep(5.0)

    # ---- Open ----
    rospy.loginfo("Opening gripper...")
    send_goal(client, 100.0) # 100 = open
    rospy.sleep(1.0)

    rospy.loginfo("Done!")

if __name__ == "__main__":
    main()
























# #!/usr/bin/env python3
# import rospy
# import tkinter as tk
# from baxter_interface import Gripper

# class GripperGUI:
#     def __init__(self, root, arm='left'):
#         self.root = root
#         self.arm = arm
#         self.root.title(f"Baxter {arm.capitalize()} Gripper Controller")

#         rospy.init_node(f"{arm}_gripper_gui")

#         self.gripper = Gripper(arm)
#         rospy.sleep(0.5)

#         # Calibrate if needed
#         if not self.gripper.calibrated():
#             print("Calibrating gripper...")
#             self.gripper.calibrate()
#             rospy.sleep(2.0)

#         # Slider to control gripper position
#         self.scale = tk.Scale(root, from_=100, to=0, length=300,
#                               label="Gripper Position (%)",
#                               orient="vertical",
#                               command=self.update_gripper)
#         self.scale.set(self.gripper.position())
#         self.scale.pack(padx=20, pady=10)

#         # Label to show current position
#         self.pos_label = tk.Label(root, text=f"Current Position: {self.gripper.position():.1f} %")
#         self.pos_label.pack(pady=10)

#         # Periodic update of position
#         self.update_position_label()

#         # Exit button
#         tk.Button(root, text="Quit", command=self.quit).pack(pady=10)

#     def update_gripper(self, value):
#         """Move gripper to desired position (0–100%)"""
#         try:
#             position = float(value)
#             self.gripper.command_position(position)
#         except Exception as e:
#             print(f"Error sending command: {e}")

#     def update_position_label(self):
#         """Continuously refresh displayed position"""
#         pos = self.gripper.position()
#         self.pos_label.config(text=f"Current Position: {pos:.1f} %")
#         self.root.after(200, self.update_position_label)

#     def quit(self):
#         print("Closing GUI...")
#         self.root.destroy()

# if __name__ == "__main__":
#     root = tk.Tk()
#     app = GripperGUI(root, arm='left')  # or 'right'
#     root.mainloop()
