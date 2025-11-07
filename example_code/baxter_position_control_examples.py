#!/usr/local/bin/python
#system level imports
import sys, os
import numpy as np
import scipy.io as sio
import time

#needed modules and classes that are specific to baxter control
from math import pi
from baxter_interface.limb import Limb
from rad_baxter_limb import RadBaxterLimb
from baxter_pykdl import baxter_kinematics as b_kin
import rospy
import tf


if __name__ == '__main__':

    # this code is specific to the way we are communicating over the
    # network, you could name it anything you want
    rospy.init_node('me_537_project')

    # here we instantiate a baxter limb object that can allow us to
    # get data and set commanded joint angles. Use 'right' if you want to
    # be able to control the right arm.
    limb = RadBaxterLimb('left')

    # this sets the maximum allowable speed, can go up to 1 or down to 0, be careful please
    limb.set_joint_position_speed(0.5)

    # this defines how long we wait to send a new command in Hz
    control_rate = rospy.Rate(500)


    # these vectors can be used for recording data if you want
    q = []
    q_dot = []
    t = []

    # baxter will time-out for safety reasons if we don't continually
    # send a command, so this loop allows us to send commands at the
    # specified rate
    while not rospy.is_shutdown():

        # you would need to put your code here to change the joint
        # angle command which for this example is set to zero
        limb.set_joint_positions_mod([0]*7)

        # this code could be used to record joint angles and velocities if needed
        q.append(limb.get_joint_angles())
        q_dot.append(limb.get_joint_velocities())
        t.append(time.time())

        # we can also use code to get his FK. This function returns a vector of
        # length 3 which is the position and a vector of length 4 which is the
        # quaternion orientation. The quaternion has the vector part first and
        # the scalar part last. It doesn't take an input argument because it
        # uses the current joint states of the robot.
        limb.get_kdl_forward_position_kinematics()

        # to get inverse kinematics, we can use the function below. However, as
        # noted in the materials I posted on LS, this only succeeds at finding
        # a solution 60ish% of the time. So if this isn't working for you,
        # consider using something else. Given the example below, you can also
        # supposedly pass a third argument of initial joint angles which seeds
        # the joint position to search from. This could be your current
        # position, or something else. You can take the result from this and
        # command is as joint angles instead of the hard coded example given
        # above.
        desired_pos = [0.5, -0.18, 0.21]
        desired_quaternion = [0.03085, 0.9945, 0.0561, 0.0829]
        q_des = limb.kin_kdl.inverse_kinematics(desired_pos, desired_quaternion)


        # causes the loop to sleep for 1/500 of a second (approximately)
        control_rate.sleep()

    # this code puts your data in a structure that allows you to write
    # it to a .mat file for analysis in matlab.
    data = {"q":q, "q_dot":q_dot, "t":t}
    sio.savemat('./test_data', data)
