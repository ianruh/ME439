# -*- coding: utf-8 -*-
"""
Created on Thu Dec  3 12:30:33 2020

@author: sinas
"""

class robot_PID(object, error):
        def __init__(self, error):
            self.K_p = rospy.get_param('K_p')
            self.K_i = rospy.get_param('K_i')
            self.K_d = rospy.get_param('K_d')
            self.error = error
            self.prev_error = 0
        def PIDcontrol(self):
            P = self.error
            I = I + self.error
            D = self.error - self.prev_error
            PID_val = self.K_p * P + self.K_i * I + self.K_d * D 
            self.prev_error = self.error
            return (PID_val)