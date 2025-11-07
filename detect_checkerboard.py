import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy

bridge = CvBridge()

# Checkerboard dimensions (inner corners)
CHECKERBOARD = (7, 7)
square_size = 0.0225  # meters

#Import K and D
import yaml
with open('camera_calibration.yaml', 'r') as f:
    calib_data = yaml.safe_load(f)
K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
D = np.array(calib_data['distortion_coefficents']['data'])

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    if ret:
        # Refine corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
        cv2.imshow('Detected Checkerboard', frame)
        cv2.waitKey(1)

    # 3D points in checkerboard frame
    objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
    objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)
    objp *= square_size

    # Given camera matrix K and distortion D
    ret, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
    #K and D come from the distortion file
    R, _ = cv2.Rodrigues(rvec)
    
    #Transfrom and Publish Pose
    import tf2_ros
    import geometry_msgs.msg
    import tf

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "head_camera"
    t.child_frame_id = "checkerboard"
    t.transform.translation.x = tvec[0]
    t.transform.translation.y = tvec[1]
    t.transform.translation.z = tvec[2]
    # convert rotation
    quat = tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([R, tvec]), [0,0,0,1]]))
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
    br.sendTransform(t)

    #to transform to base frame
    # rosrun tf tf_echo base checkerboard



   
