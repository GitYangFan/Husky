#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist

def straight(linear_speed):
    # initialize the node
    rospy.init_node('basic_motion_node', anonymous=True)
    
    # publisher
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # setup the desired linear speed in m/s
    move_cmd = Twist()
    move_cmd.linear.x = linear_speed
    move_cmd.angular.z = 0

    rospy.loginfo("Robot is going straight... Press Ctrl+C to stop.")
    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()
        
def rotate(angular_speed):
    # initialize the node
    rospy.init_node('basic_motion_node', anonymous=True)
    
    # publisher
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # setup the desired angular speed in rad/s
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = angular_speed
    
    rospy.loginfo("Robot is rotating... Press Ctrl+C to stop.")
    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()
        
def move_in_circle(linear_speed, radius):
    # initialize the node
    rospy.init_node('basic_motion_node', anonymous=True)
    
    # publisher
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # calculate the corresponding angular speed
    angular_speed = linear_speed / radius

    # setup the linear and angular speed
    move_cmd = Twist()
    move_cmd.linear.x = linear_speed
    move_cmd.angular.z = angular_speed

    rospy.loginfo("Robot is moving in a circle... Press Ctrl+C to stop.")
    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

def switch_mode(case):
    if case == 1:
        straight(linear_speed=0.1)
    elif case == 2:
        rotate(angular_speed=np.pi/5)
    elif case == 3:
        move_in_circle(linear_speed=0.1, radius=1)
    else:
        print('not a valid movement mode!')
        
if __name__ == '__main__':
    try:
        switch_mode(3)
    except rospy.ROSInterruptException:
        pass
