#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
# from IK import joint_angles

from sympy import symbols, cos, sin, Matrix, pi, sqrt
from scipy.linalg import pinv
import matplotlib.pyplot as plt
import numpy as np

global joint_angles
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
a1, a2, a3, a4, a5, a6 = symbols('a1 a2 a3 a4 a5 a6')
d1, d2, d3, d4, d5, d6 = symbols('d1 d2 d3 d4 d5 d6')

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = -pi/2, 0, -pi/2, -pi/2, pi/2, 0
a1, a2, a3, a4, a5, a6 = 50, 200, -60, 0, 0, 0
d1, d2, d3, d4, d5, d6 = 290, 0, 0, 87.5, 0, 130
theta1, theta2, theta3, theta4, theta5, theta6 = 0, pi/2, 0, 0, 0, 0
theta1_initial,theta2_initial,theta3_initial,theta4_initial,theta5_initial,theta6_initial= 0,-pi/2,0,0,0,0
radius = 30
dt = 0.01
num_steps = 100
# duration = 10.0

def circleTrajectory(r, x, y, z):
    radius = r
    theta_values = np.linspace(np.pi/2, 2*np.pi+np.pi/2, num_steps)
    x_values = np.zeros(num_steps)
    y_values = np.zeros(num_steps)
    z_values = np.zeros(num_steps)

    for i in range(num_steps):
        theta = theta_values[i]
        x_values[i] = x
        y_values[i] = y + radius * np.cos(theta)
        z_values[i] = z + radius * np.sin(theta)

    return theta_values, x_values, y_values, z_values

def getTransformationMatrix(theta, d, a, alpha):
    R_z_theta = Matrix([[cos(theta), -sin(theta), 0, 0],
                 [sin(theta), cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    T_z_d = Matrix([[1, 0, 0, 0],
                 [0, 1, 0, 0],
                 [0, 0, 1, d],
                 [0, 0, 0, 1]])

    T_x_a = Matrix([[1, 0, 0, a],
                 [0, 1, 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

    R_x_alpha = Matrix([[1, 0, 0, 0],
                 [0, cos(alpha), -sin(alpha), 0],
                 [0, sin(alpha), cos(alpha), 0],
                 [0, 0, 0, 1]])
    T = R_z_theta * T_z_d * T_x_a * R_x_alpha
    return T

def computeJacobian(theta):
    A1 = getTransformationMatrix(theta[0], d1, a1, alpha1)
    A2 = getTransformationMatrix(theta[1], d2, a2, alpha2)
    A3 = getTransformationMatrix(theta[2], d3, a3, alpha3)
    A4 = getTransformationMatrix(theta[3], d4, a4, alpha4)
    A5 = getTransformationMatrix(theta[4], d5, a5, alpha5)
    A6 = getTransformationMatrix(theta[5], d6, a6, alpha6)
    
    T01 = A1
    T02 = A1 * A2 
    T03 = A1 * A2 * A3 
    T04 = A1 * A2 * A3 * A4 
    T05 = A1 * A2 * A3 * A4 * A5
    T06 = A1 * A2 * A3 * A4 * A5 * A6
    
    T = T06

    c = [[0],[0],[1]]
    J01 = np.cross(c, T06[:3, -1], axis=0)
    J02 = np.cross(T01[:3, 2],(T06[:3,-1]-T01[:3,-1]), axis=0)
    J03 = np.cross(T02[:3, 2],(T06[:3,-1]-T02[:3,-1]), axis=0)
    J04 = np.cross(T03[:3, 2],(T06[:3,-1]-T03[:3,-1]), axis=0)
    J05 = np.cross(T04[:3, 2],(T06[:3,-1]-T04[:3,-1]), axis=0)
    J06 = np.cross(T05[:3, 2],(T06[:3,-1]-T05[:3,-1]), axis=0)
    J11 = c
    J12 = T01[:3, 2]
    J13 = T02[:3, 2]
    J14 = T03[:3, 2]
    J15 = T04[:3, 2]
    J16 = T05[:3, 2]

    J0 = np.stack([J01, J02, J03, J04, J05, J06], axis = 1)
    J0_f = np.squeeze(J0)
    J1 = np.stack([J11, J12, J13, J14, J15, J16], axis = 1)
    J1_f = np.squeeze(J1)
    J = np.concatenate((J0_f, J1_f))

    return T, J


def move_to_points():


    rate_joint = rospy.Rate(6)


    while (not rospy.is_shutdown()):
        for i in range(len(joint_angles[1])):
            if i==0: 
                continue
            [q1, q2, q3, q4, q5, q6] = joint_angles[:, i]
            print(f"Iteration number {i}")
            print(q1, q2, q3, q4, q5, q6)
            pub_joint1_pos.publish(q1)
            rate_joint.sleep()

            pub_joint2_pos.publish(q2)
            rate_joint.sleep()

            pub_joint3_pos.publish(q3)
            rate_joint.sleep()

            pub_joint4_pos.publish(q4)
            rate_joint.sleep()

            pub_joint5_pos.publish(q5)
            rate_joint.sleep()

            pub_joint6_pos.publish(q6)
            rate_joint.sleep()


if __name__ == '__main__':
    try:
        theta_values, x_values, y_values, z_values = circleTrajectory(radius, 267.5, 0, 430)
        vx_values = np.zeros(num_steps)
        vy_values = np.zeros(num_steps)
        vz_values = np.zeros(num_steps)
        current_joint_angles = np.array([theta1+theta1_initial, theta2+theta2_initial, theta3+theta3_initial, theta4+theta4_initial, theta5+theta5_initial, theta6+theta6_initial]) 
        # current_joint_angles = np.array([theta1, theta2, theta3, theta4, theta5, theta6]) 

        # Compute x, y, z velocities for each point on the circle
        for i in range(num_steps):
            theta = theta_values[i]
            vx_values[i] = 0
            vy_values[i] = -radius * np.sin(theta) * 6.28
            vz_values[i] = radius * np.cos(theta) * 6.28

        joint_velocities = np.zeros((6, len(vx_values)))
        joint_angles = np.zeros((6, len(vx_values)))
        end_effector_positions = []

        for i in range(len(vx_values)):
            T, J = computeJacobian(current_joint_angles)
            desired_velocity = np.array([vx_values[i], vy_values[i], vz_values[i], 0, 0, 0])
            J_pinv = pinv(J.astype(float))
            joint_velocities[:, i] = np.dot(J_pinv, desired_velocity)
            new_joint_angles = current_joint_angles + joint_velocities[:, i]*dt
            joint_angles[:, i] = new_joint_angles
            current_joint_angles = new_joint_angles
            end_effector_position = T[:3,3]
            end_effector_positions.append(end_effector_position)

        end_effector_positions_array = np.array(end_effector_positions)

        X = end_effector_positions_array[:, 0]
        Y = end_effector_positions_array[:, 1]
        Z = end_effector_positions_array[:, 2]

        # inititalizing node
        rospy.init_node('move_robot', anonymous=False)
        
        # robot arm joint position publishers
        pub_joint1_pos = rospy.Publisher('/chitlits_trial/joint1_control/command', Float64, queue_size=10) 
        pub_joint2_pos = rospy.Publisher('/chitlits_trial/joint2_control/command', Float64, queue_size=10) 
        pub_joint3_pos = rospy.Publisher('/chitlits_trial/joint3_control/command', Float64, queue_size=10) 
        pub_joint4_pos = rospy.Publisher('/chitlits_trial/joint4_control/command', Float64, queue_size=10) 
        pub_joint5_pos = rospy.Publisher('/chitlits_trial/joint5_control/command', Float64, queue_size=10) 
        pub_joint6_pos = rospy.Publisher('/chitlits_trial/joint6_control/command', Float64, queue_size=10)

        move_to_points()

    except rospy.ROSInterruptException: 
        pass