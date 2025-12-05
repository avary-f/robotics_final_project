#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray

def publish_checkerboard():
    rospy.init_node("checkerboard_mesh_publisher")
    pub = rospy.Publisher("checkerboard_marker", MarkerArray, queue_size=1)

    marker_array = MarkerArray()
    square_size = 0.024
    rows, cols = 8, 8

    for i in range(rows):
        for j in range(cols):
            m = Marker()
            m.header.frame_id = "checkerboard"   # Must match TF frame
            m.id = i * cols + j
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.scale.x = square_size
            m.scale.y = square_size
            m.scale.z = 0.01
            m.pose.position.x = i * square_size
            m.pose.position.y = j * square_size
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            if (i + j) % 2 == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 1, 1, 1, 1
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0, 0, 0, 1

            marker_array.markers.append(m)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        for m in marker_array.markers:
            m.header.stamp = t
        pub.publish(marker_array)
        rate.sleep()

if __name__ == "__main__":
    publish_checkerboard()
