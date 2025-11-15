#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def publish_mesh():
    pub = rospy.Publisher("checkerboard_marker", Marker, queue_size=1)
    rospy.init_node("checkerboard_mesh_publisher")

    marker = Marker()
    marker.header.frame_id = "checkerboard"   # Must match TF frame from your pose publisher
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = "package://robotics_final_project/meshes/checkerboard.obj"
    marker.mesh_use_embedded_materials = True
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.pose.orientation.w = 1.0

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        r.sleep()

if __name__ == "__main__":
    publish_mesh()
