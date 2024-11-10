#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist


def straight(linear_speed, distance):
    # initialize the node
    rospy.init_node('basic_motion_node', anonymous=True)

    # publisher
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # setup the desired linear speed in m/s
    move_cmd = Twist()
    move_cmd.linear.x = linear_speed
    move_cmd.angular.z = 0

    # setup the clock
    while rospy.Time.now().to_sec() == 0:
        rospy.loginfo("Waiting for ROS time to initialize...")
        rate.sleep()
    start_time = rospy.Time.now()

    # publish the cmd_vel
    rospy.loginfo("Robot is going straight... Press Ctrl+C to stop.")
    while not (rospy.is_shutdown() or (rospy.Time.now() - start_time).to_sec() > (distance / abs(linear_speed))):
        pub.publish(move_cmd)
        print('running', (rospy.Time.now() - start_time).to_sec(), 's')
        rate.sleep()


def rotate(angular_speed, angle):
    # initialize the node
    rospy.init_node('basic_motion_node', anonymous=True)

    # publisher
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # setup the desired angular speed in rad/s
    move_cmd = Twist()
    move_cmd.linear.x = 0
    move_cmd.angular.z = angular_speed

    # setup the clock
    while rospy.Time.now().to_sec() == 0:
        rospy.loginfo("Waiting for ROS time to initialize...")
        rate.sleep()
    start_time = rospy.Time.now()

    # publish the cmd_vel
    rospy.loginfo("Robot is rotating... Press Ctrl+C to stop.")
    while not (rospy.is_shutdown() or (rospy.Time.now() - start_time).to_sec() > (angle / abs(angular_speed))):
        pub.publish(move_cmd)
        print('running', (rospy.Time.now() - start_time).to_sec(), 's')
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
        straight(linear_speed=-0.1, distance=0.5)
    elif case == 2:
        rotate(angular_speed=np.pi / 6, angle=np.pi / 2)
    elif case == 3:
        move_in_circle(linear_speed=0.1, radius=1)
    else:
        print('not a valid movement mode!')


if __name__ == '__main__':
    try:
        switch_mode(1)
    except rospy.ROSInterruptException:
        pass
