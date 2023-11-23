#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
    
def move_to_points():

    rate_q1 = rospy.Rate(1)
    rate_q2 = rospy.Rate(2)
    
    q1 = np.arange(0, 6.283, 0.5)
    q2 = np.arange(0, 6.283, 0.5)
    
    while (not rospy.is_shutdown()):

        # for i in range(len(q1)):
        for q1 in np.arange(0, 6.283, 0.5):
            for q2 in np.arange(0, 6.283, 0.5):
                print(q1, q2)
                pub_joint2_pos.publish(q2)
                rate_q2.sleep()
            pub_joint1_pos.publish(q1)
            rate_q1.sleep()


if __name__ == '__main__':
    try:

        # inititalizing node
        rospy.init_node('move_robot', anonymous=False)
        
        # robot arm joint position publishers
        pub_joint1_pos = rospy.Publisher('/chitlits_trial/joint1_control/command', Float64, queue_size=10) 
        pub_joint2_pos = rospy.Publisher('/chitlits_trial/joint2_control/command', Float64, queue_size=10)

        move_to_points()

    except rospy.ROSInterruptException: 
        pass