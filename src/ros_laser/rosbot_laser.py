# -*- coding: utf-8 -*-
"""
Created on Fri Nov 27 19:30:16 2020

@author: sinas
"""

from sensor_msgs.msg import LaserScan

class Rosbot():
    def __init__(self):
        
        self.rosbot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0
        self.d = 0.0
        self.ctrl_c = False
        
        self.rate = rospy.Rate(10) #10 hz
        rospy.on_shutdown(self.shutdownhook)
        
    def scan_callback(self, msg):
        self.a = msg.ranges[180]
        self.b = msg.ragnes[len(msg.ragnes)/2]
        self.c = msg.ranges[540]
        self.d = msg.ranges[1]
        
        
    def read_laser(self):
        while not self.ctrl_c:
            if self.b > 5:
                self.b=5
            if self.a > 5:
                self.a = 5
            if self.c > 5:
                self.c = 5
             
            print(msg.ranges)
            print("a = "+str(self.a)+"b = " + str(self.b) + "c = " + str(self.c) + "d = " + str(self.d))
    
    def shutdownhook(self):
        self.ctrl_c = True
        
if __name__ == '__main__':
    rospy.init_node('rosbot_test', anonymous = True)
    rosbot_object = Rosbot()
    
    try:
        rosbot_object.read_laser()
        
    except rospy.ROSInterruptException:
        pass
    