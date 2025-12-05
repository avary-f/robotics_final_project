#!/usr/bin/env python3
import os
import yaml
import rospy
import rospkg
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tf
import tf2_ros
import geometry_msgs.msg

# --- Checkerboard parameters ---
CHECKERBOARD = (7, 6)   # inner corners
SQUARE_SIZE = 0.024     # meters

# Prepare object points (3D points in checkerboard frame)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

bridge = CvBridge()
br = tf2_ros.TransformBroadcaster()
board_pose = None  # Will hold (rvec, tvec) after calibration/loading

# --- File paths ---
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('chess_master')
calib_file = os.path.join(pkg_path, 'config', 'camera_calibration.yaml')
board_pose_file = os.path.join(pkg_path, 'config', 'board_pose.yaml')

# --- Load camera calibration ---
with open(calib_file, 'r') as f:
    calib_data = yaml.safe_load(f)
K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
D = np.array(calib_data['distortion_coefficents']['data'])

def save_board_pose(rvec, tvec):
    data = {'rvec': rvec.tolist(), 'tvec': tvec.tolist()}
    with open(board_pose_file, 'w') as f:
        yaml.dump(data, f)
    rospy.logwarn(f"Saved board pose to: {board_pose_file}")

def load_board_pose():
    if not os.path.exists(board_pose_file):
        rospy.logerr("board_pose.yaml does NOT exist. Run calibration first.")
        return None
    with open(board_pose_file, 'r') as f:
        data = yaml.safe_load(f)
    rvec = np.array(data['rvec'], dtype=float).reshape(3,1)
    tvec = np.array(data['tvec'], dtype=float).reshape(3,1)
    rospy.loginfo(f"Loaded board pose from: {board_pose_file}")
    return rvec, tvec

def image_callback(msg):
    """Calibration callback: detect checkerboard and save pose once."""
    global board_pose
    if board_pose is not None:
        return  # Already calibrated

    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CvBridge error: {e}")
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if not ret:
        rospy.loginfo_throttle(2, "Searching for checkerboard...")
        cv2.imshow("Checkerboard Calibration", frame)
        cv2.waitKey(1)
        return

    rospy.logwarn("FOUND CHECKERBOARD!")
    corners2 = cv2.cornerSubPix(
        gray, corners, (11,11), (-1,-1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )

    success, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
    if not success:
        rospy.logerr("solvePnP failed!")
        return

    board_pose = (rvec, tvec)
    save_board_pose(rvec, tvec)
    rospy.logwarn("Calibration complete!")

    # Show visual confirmation
    cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
    cv2.putText(frame, "BOARD DETECTED", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 3)
    cv2.imshow("Checkerboard Calibration", frame)
    cv2.waitKey(1)

def publish_board_tf(event=None):
    """Publish the checkerboard transform at a fixed rate."""
    if board_pose is None:
        return
    rvec, tvec = board_pose

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    quat = tf.transformations.quaternion_from_matrix(T)

    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "head_camera"
    t.child_frame_id = "checkerboard"
    t.transform.translation.x = float(tvec[0])
    t.transform.translation.y = float(tvec[1])
    t.transform.translation.z = float(tvec[2])
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)

def main():
    global board_pose   # declare global at the top
    rospy.init_node("checkerboard_pose_publisher")
    calibrate = rospy.get_param("~calibrate_board", False)
    rospy.loginfo(f"Calibration mode: {calibrate}")

    if calibrate:
        rospy.Subscriber("/cameras/head_camera/image", Image, image_callback, queue_size=1)
        rospy.logwarn("Waiting for checkerboard calibration...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and board_pose is None:
            rate.sleep()
        cv2.destroyAllWindows()
    else:
        board_pose = load_board_pose()
        if board_pose is None:
            rospy.logerr("Cannot load board pose. Exiting.")
            return

    rospy.Timer(rospy.Duration(0.1), publish_board_tf)
    rospy.loginfo("Publishing checkerboard TF at fixed rate...")
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
