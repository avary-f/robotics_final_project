#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import tf
import tf2_ros
import geometry_msgs.msg
import yaml
import rospkg
import os
import signal
import sys

# --- Load camera calibration ---
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('chess_master')
calib_file = os.path.join(pkg_path, 'config', 'camera_calibration.yaml')

with open(calib_file, 'r') as f:
    calib_data = yaml.safe_load(f)

K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
D = np.array(calib_data['distortion_coefficents']['data'])

# --- Checkerboard setup ---
CHECKERBOARD = (7, 7)   # inner corners
SQUARE_SIZE = 0.024     # meters

# Prepare object points (3D points of checkerboard corners)
objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

bridge = CvBridge()
br = tf2_ros.TransformBroadcaster()

# Handle Ctrl+C cleanly
def signal_handler(sig, frame):
    cv2.destroyAllWindows()
    rospy.signal_shutdown("User Exit")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {}".format(e))
        return

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        success, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
        if not success:
            rospy.logwarn_throttle(5, "solvePnP failed")
            return

        # Convert rotation vector to quaternion
        R, _ = cv2.Rodrigues(rvec)
        T_mat = np.eye(4)
        T_mat[:3, :3] = R
        T_mat[:3, 3] = tvec.flatten()
        quat = tf.transformations.quaternion_from_matrix(T_mat)

        # Publish TF using current time to avoid future extrapolation
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
        rospy.loginfo_throttle(2, "Publishing checkerboard transform")

        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
    else:
        rospy.loginfo_throttle(5, "Checkerboard not detected")

    # Show the image
    cv2.imshow("Checkerboard Detection", frame)
    cv2.waitKey(1)

def main():
    rospy.init_node("checkerboard_pose_publisher")
    rospy.Subscriber("/cameras/head_camera/image", Image, image_callback, queue_size=1)
    rospy.loginfo("Checkerboard pose publisher started")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
