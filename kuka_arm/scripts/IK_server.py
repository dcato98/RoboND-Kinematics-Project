#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya
# Revision History:
#   Aug 6, 2017 - David Cato - modify to pass submission in Udacity Pick and Place project

from __future__ import print_function
from geometry_msgs.msg import Pose
from kuka_arm.srv import *
from sympy import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import dill as pickle
import numpy as np
import os
import rospy


#########################
### Utility Functions ###
#########################
def DH_transform(al, a, d, q):
    """
    Returns the homogeneous transform given Denavit-Hartenberg parameters.
    `al` is the angle between z0 and z1 about x0
    `a` is the distance from z0 to z1 along x0
    `d` is the distance from x0 to x1 along z1
    `q` is the angle between x0 and x1 about z1
    """
    transform = Matrix([[        cos(q),        -sin(q),        0,          a],
                        [sin(q)*cos(al), cos(q)*cos(al), -sin(al), -sin(al)*d],
                        [sin(q)*sin(al), cos(q)*sin(al),  cos(al),  cos(al)*d],
                        [             0,              0,        0,          1]])
    return transform

def rot3D(axis, angle):
    """
    Returns a 3x3 rotation Matrix about the specified axis.
    """
    if axis == 'x':
        m = Matrix([[1,          0,           0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle),  cos(angle)]])
    elif axis == 'y':
        m = Matrix([[ cos(angle), 0, sin(angle)],
                    [          0, 1,          0],
                    [-sin(angle), 0, cos(angle)]])
    elif axis == 'z':
        m = Matrix([[cos(angle), -sin(angle), 0],
                    [sin(angle),  cos(angle), 0],
                    [         0,           0, 1]])
    else:
        raise ValueError("`axis` must be 'x', 'y', or 'z'")
    return m

def forward_kinematics(transform, manipulator_map, origin_xyz=[0., 0., 0.]):
    """
    Computes forward kinematics for a serial manipulator.
    Parameters:
        `transform`: symbolic transform from the origin to the end position
        `manipulator_map`: dictionary of symbolic variables for substitution in the transform
        `origin_xyz`: list of coordinates specifying the origin in the form [x, y, z]
    Returns the end position of the manipulator relative to the origin.
    """
    # compute position relative to origin
    origin = Matrix(origin_xyz + [1.0])
    end_pos = transform.evalf(subs=manipulator_map) * origin
    # convert to list
    end_pos = end_pos.transpose().tolist()[0][0:3]
    return end_pos

def R_from_T(transform):
    """Extracts rotation matrix from transform matrix"""
    rotation = transform[0:3, 0:3]
    return rotation

#################################
### Inverse Kinematics Server ###
#################################
g_symbols = None
def handle_calculate_IK(req):
    """
    Handler for calculating inverse kinematics (IK) for the Kuka KR210 6-DOF manipulator.
    """
    n_poses = len(req.poses)
    rospy.loginfo("Received {0} eef-poses from the plan".format(n_poses))
    
    # validate request
    if n_poses < 1:
        print("No valid poses received")
        return -1
    
    # Manipulator Specs
    manipulator_name = 'Kuka KR210'
    pickled_sym_path = '/home/robond/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts/{0}_syms.pkl'.format(manipulator_name.replace(' ', '_'))
    n_joints = 7  # 6 revolute joints + 1 translation to center of gripper_link
    
    # Try to retrieve syms from cache
    global g_symbols
    if g_symbols:
        (dh, al, a, d, q, r, p, y, ee_x, ee_y, ee_z, Ti_iplus1, T0_Joint, R3_EE_sym, WC_sym) = g_symbols
    # Next try loading syms from dill-pickled file
    elif os.path.exists(pickled_sym_path):
        print("Loading symbolic variables from {0}".format(pickled_sym_path))
        (dh, al, a, d, q, r, p, y, ee_x, ee_y, ee_z, Ti_iplus1, T0_Joint, R3_EE_sym, WC_sym) = pickle.load(open(pickled_sym_path, 'rb'))
    # Compute the syms
    else:
        print("Calculating symbolic variables, this may take some time.")
        # Kuka KR210 DH parameters
        al = symbols('al{0}:{1}'.format(0, n_joints)) # alpha_i-1
        a = symbols('a{0}:{1}'.format(0, n_joints))
        d = symbols('d{0}:{1}'.format(1, n_joints+1))
        q = symbols('q{0}:{1}'.format(1, n_joints+1)) # theta_i
        r, p, y = symbols('r p y')
        (ee_x, ee_y, ee_z) = symbols('ee_x, ee_y, ee_z')
    
        # DH parameter table
        #     alpha_i        a_i           d_i-1        theta_i-1
        dh = {al[0]:      0, a[0]:      0, d[0]:  0.75, q[0]:       q[0], 
              al[1]: -pi/2., a[1]:   0.35, d[1]:     0, q[1]: q[1]-pi/2., 
              al[2]:      0, a[2]:   1.25, d[2]:     0, q[2]:       q[2], 
              al[3]: -pi/2., a[3]: -0.054, d[3]:   1.5, q[3]:       q[3], 
              al[4]:  pi/2., a[4]:      0, d[4]:     0, q[4]:       q[4], 
              al[5]: -pi/2., a[5]:      0, d[5]:     0, q[5]:       q[5], 
              al[6]:      0, a[6]:      0, d[6]: 0.303, q[6]:          0}
    
        # individual homogeneous transforms and compositions of homogeneous transforms
        # T0_Joint[i] is the composed transform from the base_link to joint_i
        Ti_iplus1 = [None]*n_joints
        T0_Joint = [1] + [None]*n_joints
        for i in range(n_joints):
            # homogeneous transform from joint i to i+1
            Ti_iplus1[i] = DH_transform(al[i], a[i], d[i], q[i]).subs(dh)
            # composition of homogeneous transforms from base_link to joint_i
            T0_Joint[i+1] = simplify(T0_Joint[i] * Ti_iplus1[i])

        # symbolic Euler angle rotation matrix
        R_rpy = simplify(rot3D('z', y) * rot3D('y', p) * rot3D('x', r))

        # correction to account for orientation difference between definition of gripper link in URDF vs DH Convention
        R_corr = simplify(rot3D('z', pi) * rot3D('y', -pi/2.))
        R_EE = R_rpy * R_corr
        
        # base_link to wrist center rotation matrix
        R0_3 = R_from_T(T0_Joint[3])
        
        # rotation matrix from wrist center to end-effector
        R3_EE_sym = R0_3.transpose() * R_EE # can use transpose() instead of inv("LU") here since matrix is orthogonal

        # vector location of wrist center from end-effector pose
        EE_xyz = Matrix([ee_x, ee_y, ee_z])
        WC_sym = simplify(EE_xyz - d[6].subs(dh) * R_EE[:,2])
        WC_sym = WC_sym.transpose()
        
        print("Done calculating symbolic variables.")
        
        # Cache and dill-pickle the symbols
        g_symbols = (dh, al, a, d, q, r, p, y, ee_x, ee_y, ee_z, Ti_iplus1, T0_Joint, R3_EE_sym, WC_sym)
        pickle.dump(g_symbols, open(pickled_sym_path, 'wb'))
        print("Symbolic variables dill-pickled to {0}.".format(pickled_sym_path))

    # Initialize service response
    joint_trajectory_list = []
    for pose_idx, pose in enumerate(req.poses):
        theta = [None] * 6
        
        # end-effector roll, pitch, and yaw
        EE_ori = pose.orientation
        quaternion = [EE_ori.x, EE_ori.y, EE_ori.z, EE_ori.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        
        # solve for actual wrist center from end-effector
        wc_subs = {ee_x: pose.position.x, 
                   ee_y: pose.position.y, 
                   ee_z: pose.position.z,
                   r: roll, p: pitch, y: yaw}
        WC = WC_sym.evalf(subs=wc_subs)
        
        # solve for angles from base to wrist center
        theta[0] = atan2(WC[1], WC[0])
        # SSS triangle for theta2 and theta3
        side_a = 1.501 # from where?
        side_b = sqrt(pow((sqrt(WC[0]**2 + WC[1]**2) - a[1].subs(dh)), 2) + pow((WC[2] - d[0].subs(dh)), 2))
        side_c = a[2].subs(dh)
        angle_a = acos((side_b**2 + side_c**2 -side_a**2) / (2 * side_b * side_c))
        angle_b = acos((side_c**2 + side_a**2 -side_b**2) / (2 * side_c * side_a))
        angle_c = acos((side_a**2 + side_b**2 -side_c**2) / (2 * side_a * side_b))
        theta[1] = pi / 2 - angle_a - atan2(WC[2] - d[0].subs(dh), sqrt(WC[0]**2 + WC[1]**2) - a[1].subs(dh))
        theta[2] = pi / 2 - (angle_b + 0.036) # from where? accounts for sag in link4 of -0.054m
        
        # solve for rotation matrix from wrist center to end-effector
        r3_ee_subs = {q[0]:theta[0], 
                      q[1]:theta[1], 
                      q[2]:theta[2],
                      r: roll, p: pitch, y: yaw}
        R3_EE = R3_EE_sym.evalf(subs=r3_ee_subs)

        # solve for angles from wrist center to end effector
        theta[4] = atan2(sqrt(R3_EE[0,2]**2 + R3_EE[2,2]**2), R3_EE[1,2])
        # reduce unnecessary rotations
        if sin(theta5) < 0:
            theta[3] = atan2(-R3_EE[2,2], R3_EE[0,2])
            theta[5] = atan2(R3_EE[1,1], -R3_EE[1,0])
        else:
            theta[3] = atan2(R3_EE[2,2], -R3_EE[0,2])
            theta[5] = atan2(-R3_EE[1,1], R3_EE[1,0])
        
        # Populate response for the IK request
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = list([theta[i] for i in range(len(theta))])
        joint_trajectory_list.append(joint_trajectory_point)

    rospy.loginfo("length of Joint Trajectory List: {0}".format(len(joint_trajectory_list)))
    return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
