#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
title: Motor Node - ME439 Intro to robotics, wisc.edu
"""

import rospy
from std_msgs.msg import Float32
# MAX_SPEED is 480 (hard-coded)
from pololu_drv8835_rpi import motors, MAX_SPEED
import time
import numpy as np


class Motor:
    """
    Allow to abstract away some of the details of the motors.
    """

    def __init__(self, motor, position_rad_topic, radius=0.02, direction_correction=1.0):
        """
        Params:
            - motor: The motor to control
            - position_rad_topic: the ros topic where the position is published in radians
            - radius: the radius of the wheel in meters
            - direction_correction: a multiplier for the speed to correct a direction inversion
        """
        self.motor = motor
        self.position_rad_topic = position_rad_topic
        self.radius = radius
        self.direction_correction = direction_correction

        # Initialize the current state
        self.speed_rad = 0          # rad/s
        self.speed_ms = 0           # m/s
        self.position_rad = 0       # rad
        self.position_meters = 0    # meters

        self.last_msg = None

        # Subscribe to the speed topic
        self.position_rad_sub = rospy.Subscriber(
            self.position_rad_topic, Float32, self.position_rad_callback)

    def position_rad_callback(msg):
        old_pos_rad = self.position_rad  # Save for speed calc

        # Set positions
        self.position_rad = float(msg.data)
        self.position_meters = self.position_rad * self.radius

        # calculate speeds
        if(self.last_msg == None):
            self.last_msg = time.time()
        else:
            delta = time.time() - self.last_msg
            self.last_msg = time.time()
            self.speed_rad = (self.position_rad - self.old_pos_rad) / delta
            self.speed_ms = self.speed_rad * self.radius

    def setSpeed(value):
        self.motor.setSpeed(value*self.direction_correction)

    def getSpeedRad():
        return self.speed_rad


class PIDController:
    """
    Encapsulate the PID controller code.
    """

    def __init__(self, target, kp=0.0, ki=0.0, kd=0.0):
        self.target = target
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.error_prev = 0.0
        self.error_int = 0.0

        self.last_update = time.time()

    def update(targetSpeed):
        """
        Target speed in rad/s
        """
        self.targetSpeed = targetSpeed
        dt = time.time() - self.last_update
        self.last_update = time.time()

        # Compute the error: difference between the target and the actual current state.
        error = targetSpeed - self.motor.getSpeedRad()
        self.error_int = self.error_int + dt*error
        error_derivative = (error - self.error_prev)/dt

        # We are done with the previous value of error... set that variable to the current error to remember it for next timestep.
        self.error_prev = error

        # Actually compute the Motor command
        command = self.kp*error + self.ki * self.error_int + self.kd*error_derivative
        self.motor.setSpeed(command)


class Node:
    def __init__(self):
        rospy.init_node('motor_node')

        # Subscribe to the "wheel_speed_desired" topics
        self.sub_left = rospy.Subscriber(
            '/wheel_speed_desired_left', Float32, self.set_wheel_speed_left)
        self.sub_right = rospy.Subscriber(
            '/wheel_speed_desired_right', Float32, self.set_wheel_speed_right)

        self.motor1 = Motor(motors.motor1, "/sensors_E0_proc")
        self.motor2 = Motor(motors.motor2, "/sensors_E1_proc",
                            direction_correction=-1.0)
        self.pid1 = PIDController(motor1, 1.0, 1.0, 0.0)
        self.pid2 = PIDController(motor2, 1.0, 1.0, 0.0)

        try:
            rospy.spin()    # keep the node from exiting
        except rospy.ROSInterruptException:
            motors.motor1.setSpeed(0)
            motors.motor2.setSpeed(0)
            pass

    def set_wheel_speed_left(self, msg_in):
        wheel_speed_desired_left = int(msg_in.data)
        self.pid1.update(wheel_speed_desired_left)

    def set_wheel_speed_right(self, msg_in):
        wheel_speed_desired_right = int(msg_in.data)
        self.pid2.update(wheel_speed_desired_right)


# Section to start the execution, with Exception handling.
if __name__ == "__main__":
    node = Node()
