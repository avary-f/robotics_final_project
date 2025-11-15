#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

rospy.init_node("checkerboard_mesh_pub")
pub = rospy.Publisher("checkerboard_marker", Marker, queue_size=1)

marker = Marker()
marker.header.frame_id = "checkerboard"   # <-- your transform frame
marker.header.stamp = rospy.Time.now()

marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = "package://robotics_final_project/meshes/checkerboard.obj"
marker.mesh_use_embedded_materials = True

marker.action = Marker.ADD
marker.scale.x = 1.0  # Adjust if mesh is too large or small
marker.scale.y = 1.0
marker.scale.z = 1.0

# If origin in OBJ isn’t centered, adjust here
marker.pose.orientation.w = 1.0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    rate.sleep()
