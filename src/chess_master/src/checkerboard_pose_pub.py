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
import tf2_geometry_msgs  # This registers PoseStamped with tf2


tf_buffer = None
tf_listener = None

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

#Next problems to solve. I think that it's being published in the base frame directly, but calibration happens through head frame. Need to figure out if you want to calibrate in base or add another transform to get from head_camer to base 

# def publish_board_tf(event=None):
#     """Publish the checkerboard transform at a fixed rate in both head_camera and base frames."""
#     if board_pose is None:
#         return
#     rvec, tvec = board_pose


#     # --- pose in head_camera frame ---
#     R, _ = cv2.Rodrigues(rvec)
#     T = np.eye(4)
#     T[:3, :3] = R
#     T[:3, 3] = tvec.flatten()
#     quat = tf.transformations.quaternion_from_matrix(T)

#     t_camera = geometry_msgs.msg.TransformStamped()
#     t_camera.header.stamp = rospy.Time.now()
#     t_camera.header.frame_id = "head_camera"
#     t_camera.child_frame_id = "checkerboard"
#     t_camera.transform.translation.x = float(tvec[0])
#     t_camera.transform.translation.y = float(tvec[1])
#     t_camera.transform.translation.z = float(tvec[2])
#     t_camera.transform.rotation.x = quat[0]
#     t_camera.transform.rotation.y = quat[1]
#     t_camera.transform.rotation.z = quat[2]
#     t_camera.transform.rotation.w = quat[3]

#     br.sendTransform(t_camera)  # always publish head_camera version

#     # --- transform to base frame ---
#     try:
#         # Convert to geometry_msgs/PoseStamped
#         from geometry_msgs.msg import PoseStamped
#         pose_camera = PoseStamped()
#         pose_camera.header.frame_id = "head_camera"
#         pose_camera.header.stamp = rospy.Time.now()
#         pose_camera.pose.position.x = float(tvec[0])
#         pose_camera.pose.position.y = float(tvec[1])
#         pose_camera.pose.position.z = float(tvec[2])
#         pose_camera.pose.orientation.x = quat[0]
#         pose_camera.pose.orientation.y = quat[1]
#         pose_camera.pose.orientation.z = quat[2]
#         pose_camera.pose.orientation.w = quat[3]

#         global tf_buffer  # so the function can see it
#         pose_base = tf_buffer.transform(pose_camera, "base", rospy.Duration(1.0))

#         t_base = geometry_msgs.msg.TransformStamped()
#         t_base.header.stamp = rospy.Time.now()
#         t_base.header.frame_id = "base"
#         t_base.child_frame_id = "checkerboard_base"
#         t_base.transform.translation = pose_base.pose.position
#         t_base.transform.rotation = pose_base.pose.orientation

#         br.sendTransform(t_base)

#     except tf2_ros.TransformException as ex:
#         rospy.logwarn("Transform head_camera -> base failed: {}".format(ex))


def publish_board_tf(event=None):
    """Publish the checkerboard TF from saved calibration only."""
    global board_pose
    if board_pose is None:
        return

    rvec, tvec = board_pose

    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Build 4x4 homogeneous transform
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    # Convert to quaternion
    quat = tf.transformations.quaternion_from_matrix(T)

    # Create TransformStamped message
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base"          # publish directly in base frame
    t.child_frame_id = "checkerboard"
    t.transform.translation.x = float(tvec[0])
    t.transform.translation.y = float(tvec[1])
    t.transform.translation.z = float(tvec[2])
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    # Broadcast the transform
    br.sendTransform(t)





def main():
    global board_pose, tf_buffer, tf_listener   # declare global at the top
    rospy.init_node("checkerboard_pose_pub")
    calibrate = rospy.get_param("~calibrate_board", False)
    rospy.loginfo(f"Calibration mode: {calibrate}")

    # Create TF buffer and listener AFTER init_node
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

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
