#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 11:55:48 2020

@author: Peter Adamczyk, University of Wisconsin - Madison
"""

import rospy
import numpy as np
from std_msgs.msg import Int32, Float32

# Here we will create one or more publishers for the processed topics.
# And the associated Messages.
# They are created outside the "listener" function
# because they will actually be called within the Callback function(s).
pub_A0_proc = rospy.Publisher('/sensors_A0_proc', Float32, queue_size=1)
pub_E0_proc = rospy.Publisher('/sensors_E0_proc', Float32, queue_size=1)
pub_E1_proc = rospy.Publisher('/sensors_E1_proc', Float32, queue_size=1)

# Encoder initial positions
encoder_0_initial = None
encoder_1_initial = None

# Gear box ratio
gear_ratio = 120  # rotations of motor shaft / rotations of output shaft


def sensors_listener():
    # Initialize the Node
    rospy.init_node('sensors_processor', anonymous=False)

    # Set up a listener for the topic you want
    # Remember to name a "callback" function (must define below) to handle the data
    sub_A0 = rospy.Subscriber('/sensors_A0', Int32, cal_and_pub_A0)
    sub_E0 = rospy.Subscriber('/sensors_E0', Int32, cal_and_pub_E0)
    sub_E1 = rospy.Subscriber('/sensors_E1', Int32, cal_and_pub_E1)

    # spin() to prevent the function from exiting
    rospy.spin()


def cal_and_pub_A0(msg_in):
    analog_level = float(msg_in.data)

    # calibrate the relationship
    # between the signal that comes in and the signal in real units
    # This can be from a Data Sheet, or from measurements you make yourself.

    # Example: MaxBotix LV-EZ02 says:
    # Distance in Inches = Volts / (Vcc/512).
    # For A0, Vcc is 3.3 Volts and signals are in 2^10 levels from 0 to Vcc
    # Therefore (Distance in Meters) = 0.0254 (m/in)*(distance in Inches)
    # UPDATE THESE EQUATIONS
    analog_volts = analog_level * (1.0/1.0)
    distance_meters = analog_volts / (1.0/1.0) * 1.0

    A0_proc = Float32()
    A0_proc.data = distance_meters
    pub_A0_proc.publish(A0_proc)


def cal_and_pub_E0(msg_in):
    """
    Gets the count from the encoder and published the net rotations since start rad.
    """
    global encoder_0_initial, gear_ratio

    count = float(msg_in.data)

    # Relative to the initial reading when the program starts
    if(encoder_0_initial == None):
        encoder_0_initial = count
    count_relative = count - encoder_0_initial

    # 4 ticks per motor shaft rotation
    motor_rot = count_relative / 4
    output_rot = motor_rot / gear_ratio
    output_rad = output_rot * 2 * np.pi

    E0_proc = Float32()
    E0_proc.data = output_rad
    pub_E0_proc.publish(E0_proc)


def cal_and_pub_E1(msg_in):
    """
    Gets the count from the encoder and published the net rotations since start rad.
    """
    global encoder_1_initial, gear_ratio

    count = float(msg_in.data)

    # Relative to the initial reading when the program starts
    if(encoder_1_initial == None):
        encoder_1_initial = count
    count_relative = count - encoder_1_initial

    # 4 ticks per motor shaft rotation
    motor_rot = count_relative / 4
    output_rot = motor_rot / gear_ratio
    output_rad = output_rot * 2 * np.pi

    E1_proc = Float32()
    E1_proc.data = output_rad
    pub_E1_proc.publish(E1_proc)


if __name__ == '__main__':
    try:
        sensors_listener()
    except rospy.ROSInterruptException:
        pass
