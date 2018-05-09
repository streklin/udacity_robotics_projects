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


#conversion constants
rtd = 180./pi # radians to degrees
dtr = pi/180. # degrees to radians


#get "angle c", given sides a,b, and c
def law_of_cosines(a,b,c):
    cos_c = (a*a + b*c - c*c) / (2*a*b)
    return acos(cos_c)

def create_transform(alpha, a, d, q):
    TF = Matrix([
        [cos(q),                -sin(q),                0,                            a],
        [sin(q) * cos(alpha),   cos(q) * cos(alpha),    -sin(alpha),    -sin(alpha) * d],
        [sin(q) * sin(alpha),   cos(q) * sin(alpha),    cos(alpha),      cos(alpha) * d],
        [0,                     0,                      0,                            1]
    ])

    return TF

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Forward Kinematics
        ## Your FK code here

        ## Insert IK code here!

        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        dh_params = {
            alpha0: 0, a0: 0, d1: 0.75, q1: q1,
            alpha1: -pi / 2., a1: 0.35, d2: 0, q2: -pi / 2. + q2,
            alpha2: 0, a2: 1.25, d3: 0, q3: q3,
            alpha3: -pi / 2., a3: -0.054, d4: 1.50, q4: q4,
            alpha4: pi / 2., a4: 0, d5: 0, q5: q5,
            alpha5: -pi / 2., a5: 0, d6: 0, q6: q6,
            alpha6: 0, a6: 0, d7: 0.303, q7: 0
        }

        # Define Modified DH Transformation matrix
        T_01 = create_transform(alpha0, a0, d1, q1).subs(dh_params)
        T_12 = create_transform(alpha1, a1, d2, q2).subs(dh_params)
        T_23 = create_transform(alpha2, a2, d3, q3).subs(dh_params)
        T_34 = create_transform(alpha3, a3, d4, q4).subs(dh_params)
        T_45 = create_transform(alpha4, a4, d5, q5).subs(dh_params)
        T_56 = create_transform(alpha5, a5, d6, q6).subs(dh_params)
        T_6G = create_transform(alpha6, a6, d7, q7).subs(dh_params)

        # transform from base to end-effector
        T_0G = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6G


        #Gripper Link Correction
        r, p, y = symbols('r p y')

        Rot_X = Matrix([
            [1, 0, 0],
            [0, cos(r), -sin(r)],
            [0, sin(r), cos(r)]
        ])

        Rot_Y = Matrix([
            [cos(p), 0, sin(p)],
            [0, 1, 0],
            [-sin(p), 0, cos(p)]
        ])

        Rot_Z = Matrix([
            [cos(y), -sin(y), 0],
            [sin(y), cos(y), 0],
            [0, 0, 1]
        ])

        Rot_EE = Rot_Z * Rot_Y * Rot_X

        #180 degree rotation around z
        M1 = Matrix([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]
        ])

        #-90 degree rotation around y
        M2 = Matrix([
            [0, 0, -1],
            [0, 1, 0],
            [1, 0, 0]
        ])

        ROT_Correction = M1 * M2
        Rot_EE = Rot_EE * ROT_Correction

        ###

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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rot_EE = Rot_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            ###
            #
            # Calculate joint angles using Geometric IK method
            #
            #
            ###
            EE = Matrix([
                [px],
                [py],
                [pz]
            ])

            WC = EE - 0.303 * Rot_EE[:, 2]

            wx = WC[0]
            wy = WC[1]
            wz = WC[2]

            theta1 = atan2(WC[1], WC[0])

            # law of cosines from diagram in lesson
            side_a = 1.5
            side_b = sqrt(pow(sqrt(wx* wx + wy * wy) - 0.35, 2) + pow(wz - 0.75, 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))

            #not sure why, but occasionally getting imaginary values for angle_a, and angle_b - taking the real parts
            angle_a = re(angle_a)
            angle_b = re(angle_b)

            theta2 = pi / 2 - angle_a - atan2(wz - 0.75, sqrt(wx * wx + wy * wy) - 0.35)
            theta3 = pi / 2 - (angle_b)


            # save ourselves a bit of calculation (and arm movement) by only move the risk when we are in the correct location
            if x == len(req.poses) - 1:
                R_03 = T_01[0:3, 0:3] * T_12[0:3, 0:3] * T_23[0:3, 0:3]
                R_03 = R_03.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
                R_36 = R_03.inv("LU") * Rot_EE

                print ("cos beta = ", R_36[1,2])

                #extract euler angles
                theta4 = atan2(R_36[2, 2], -R_36[0, 2])

                #be careful about the case when that lonely cosine becomes zero and messes everything up
                theta5 = pi / 2

                if np.abs(R_36[1,2]) > 0.005:
                    theta5 = atan2(sqrt(R_36[0, 2] * R_36[0, 2] + R_36[2, 2] * R_36[2, 2]), R_36[1, 2])

                theta6 = atan2(-R_36[1, 1], R_36[1, 0])
            else:
                theta4 = 0
                theta5 = -pi/2 #keep the hand down so it doesn't collide with anything
                theta6 = 0


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            EE = T_0G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            your_ee = [EE[0, 3], EE[1, 3], EE[2, 3]]

            ee_x_e = abs(your_ee[0] - px)
            ee_y_e = abs(your_ee[1] - py)
            ee_z_e = abs(your_ee[2] - pz)
            ee_offset = sqrt(ee_x_e ** 2 + ee_y_e ** 2 + ee_z_e ** 2)

            print ("ERROR: ", ee_offset)

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
