#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

def homo_transform (alpha, a, d, theta):
    T = Matrix([[           cos(theta),           -sin(theta),           0,             a],
                [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                    0,                     0,          0,              1]])
    return T

def rot_x (q):
    T = Matrix([[1,      0,       0, 0],
                [0, cos(q), -sin(q), 0],
                [0, sin(q),  cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_y (q):
    T = Matrix([[ cos(q), 0, sin(q), 0],
                [      0, 1,      0, 0],
                [-sin(q), 0, cos(q), 0],
                [0,      0,       0, 1]])
    return T

def rot_z (q):
    T = Matrix([[cos(q), -sin(q), 0, 0],
                [sin(q),  cos(q), 0, 0],
                [     0,       0, 1, 0],
                [0,      0,       0, 1]])
    return T

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        q1,q2,q3,q4,q5,q6,q7 = symbols("q1:8")
        d1,d2,d3,d4,d5,d6,d7 = symbols("d1:8")
        a0,a1,a2,a3,a4,a5,a6 = symbols("a0:7")
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols("alpha0:7")
        # symbol for req
        roll, pitch, yaw = symbols('r p y')
        #
        #
        # Create Modified DH parameters
        s = {alpha0: 0, a0: 0, d1: 0.75,  # 0.33 + 0.42 = 0.75
             alpha1: -pi / 2, a1: 0.35, d2: 0, q2: q2 - pi / 2,
             alpha2: 0, a2: 1.25, d3: 0,
             alpha3: -pi / 2, a3: -0.054, d4: 1.5,  # 0.96 + 0.54 = 1.5
             alpha4: pi / 2, a4: 0, d5: 0,
             alpha5: -pi / 2, a5: 0, d6: 0,
             alpha6: 0, a6: 0, d7: 0.303, q7: 0}  # 0.193 + 0.11 = 0.303
        #
        #
        # Define Modified DH Transformation matrix
        T0_1 = homo_transform(alpha0, a0, d1, q1)

        T1_2 = homo_transform(alpha1, a1, d2, q2)

        T2_3 = homo_transform(alpha2, a2, d3, q3)

        T3_4 = homo_transform(alpha3, a3, d4, q4)

        T4_5 = homo_transform(alpha4, a4, d5, q5)

        T5_6 = homo_transform(alpha5, a5, d6, q6)

        T6_G = homo_transform(alpha6, a6, d7, q7)

        #
        # Extract rotation matrices from the transformation matrices
        #
        #
        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)
        T3_4 = T3_4.subs(s)
        T4_5 = T4_5.subs(s)
        T5_6 = T5_6.subs(s)
        T6_G = T6_G.subs(s)

        #
        #
        # Create individual transformation matrices
        # correction orientation difference between urdf versus dh convertion
        # first rotate around z-axis by pi
        R_z = rot_z(pi)

        # then rotate around y-axis by -pi/2
        R_y = rot_y(-pi / 2)

        # calculate total correction factor
        Tcorr = R_z * R_y

        # symbol for req result
        T0_6 = rot_z(yaw) * rot_y(pitch) * rot_x(roll)

        # for theta4,5,6
        T0_3 = T0_1 * T1_2 * T2_3
        T3_6 = T0_3 ** -1 * T0_6

        ###
        #

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (p_roll, p_pitch, p_yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #
            #
            # Calculate joint angles using Geometric IK method
            #
            #
            # Calculate joint angles using Geometric IK method
            WC = Matrix([px, py, pz, 1]) - d7.subs(s) * (
                                                            T0_6.evalf(
                                                                subs={roll: p_roll, pitch: p_pitch, yaw: p_yaw}) * Tcorr
                                                        )[:, 2]
            wx, wy, wz, _ = WC

            # theta1
            theta1 = atan2(wy, wx)

            # theta2,3
            s1 = sqrt(wx ** 2 + wy ** 2) - a1
            s2 = wz - d1
            s3 = sqrt(s2 ** 2 + s1 ** 2)
            s4 = sqrt(a3 ** 2 + d4 ** 2)
            beta1 = atan2(s2, s1)

            D2 = (a2 ** 2 + s3 ** 2 - s4 ** 2) / (2 * a2 * s3)
            beta2 = atan2(sqrt(1 - D2 ** 2), D2)

            D3 = (a2 ** 2 + s4 ** 2 - s3 ** 2) / (2 * a2 * s4)
            beta3 = atan2(sqrt(1 - D3 ** 2), D3)

            beta4 = atan2(-a3, d4)

            theta2 = ((pi / 2) - beta2 - beta1).evalf(subs=s)
            theta3 = ((pi / 2) - beta4 - beta3).evalf(subs=s)

            # theta4, 5, 6
            R3_6_eval = T3_6.evalf(subs={roll: p_roll, pitch: p_pitch, yaw: p_yaw, q1: theta1, q2: theta2, q3: theta3})
            theta4, theta5, theta6 = tf.transformations.euler_from_matrix(np.array(R3_6_eval).astype(np.float64),
                                                                          axes='ryzx')
            theta4 = theta4 + pi
            theta6 = theta6 + pi / 2
            ###
		
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
