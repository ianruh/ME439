#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
title: Motor Command Node - ME439 Intro to robotics, wisc.edu
"""

import rospy
from std_msgs.msg import Float32


def talker_for_wheel_speeds():

    rospy.init_node('MotorCommandNode', anonymous=False)

    pub_wheel_speed_desired_left = rospy.Publisher(
        'wheel_speed_desired_left', Float32, queue_size=1)
    pub_wheel_speed_desired_right = rospy.Publisher(
        'wheel_speed_desired_right', Float32, queue_size=1)

    wheel_speed_desired_left_msg = Float32()
    wheel_speed_desired_right_msg = Float32()

    while True:
        wheel_speed_desired_left = input(
            'Enter wheel speed left (in rad/s) \n')
        wheel_speed_desired_right = input(
            'Enter wheel speed right (in rad/s) \n')
        wheel_speed_desired_left_msg.data = wheel_speed_desired_left
        wheel_speed_desired_right_msg.data = wheel_speed_desired_right

        pub_wheel_speed_desired_left.publish(wheel_speed_desired_left_msg)
        pub_wheel_speed_desired_right.publish(wheel_speed_desired_right_msg)


# Section to start the execution, with Exception handling.
if __name__ == "__main__":
    try:
        talker_for_wheel_speeds()
    except rospy.ROSInterruptException:
        pass
