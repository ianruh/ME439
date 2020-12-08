#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32
from lidar import *

class Point();
    def __init__(x, z, r):
        self.x = x
        self.z = z
        self.r = r

class Rosbot():
    def __init__(self):
        
        self.rosbot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.left_pub = rospy.Publisher('/wheel_speed_desired_left',Float32,queue_size=1)
        self.right_pub = rospy.Publisher('/wheel_speed_desired_right',Float32,queue_size=1)

        self.ctrl_c = False
        
        self.rate = rospy.Rate(10) #10 hz

        # Lidar Params
        self.base_speed = 0.1
        self.targetAngle = -1 * np.pi /2
        self.targetDistance = 0.5
        self.pid_angle = PID(P=4.0)
        self.pid_angle.setPoint(self.targetAngle)
        self.pid_distance = PID(P=10.0, I=0.0, D=0.0)
        self.pid_distance.setPoint(self.targetDistance)
        self.wheelWidth = rospy.get_param('wheel_width_actual')
        self.wheelDiameter = rospy.get_param('wheel_diameter_actual')
        self.points = []
        
        rospy.on_shutdown(self.shutdownhook)
    
    def setSpeed(self, leftSpeed, rightSpeed):
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = leftSpeed
        right_msg.data = rightSpeed
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def aroundObject(self, filteredPoints, speedSetter):
        # Calculate the angular correction
        
        # We assume the 5 closest points will be near eachother, and find the average angle they are from
        # the robot. We are going to following count-clockwise, so we want to turn until they are at ~90
        # to the left.
        closestFive = closestPoint(filteredPoints, n=5)
        angles = np.array([np.arctan2(p.x, -1*p.z) for p in closestFive])
        avgAngle = np.average(angles)
        avgDistance = np.average(np.array([p.r for p in filteredPoints]))
        omega_angle = self.pid_angle.update(avgAngle)
        omega_distance = -1 * self.pid_distance.update(avgDistance)
        omega_distance = limit(omega_distance, 4.0)
        # print("Dist: {}, Omega: {}          Angle: {}, Omega: {}".format(avgDistance, omega_distance, avgAngle, omega_angle))
        # print(omega_angle)
        # print(avgAngle)
        omega = (omega_angle + omega_distance) / 2
        # print("X: {}, Y: {}, R: {}".format(closestFive[0].x, closestFive[0].y, closestFive[0].r))
        # deltaAngle = self.targetAngle - avgAngle # negative means turn right
        # deltaDistance = self.targetDistance - avgDistance # negative mean too close
        # delta = deltaAngle - 30*deltaDistance

        vl = self.base_speed - (0.5 * omega * self.wheelWidth)
        vr = self.base_speed + (0.5 * omega * self.wheelWidth)

        speedSetter(vl, vr)
        
    def scan_callback(self,msg):
        
        ###############################################################
        #### NOTE : The RP Lidar is able to read valus > 0.135(m) #####
        ###############################################################
        # len.msg.ranges = 720
        self.points = []
        anglular_res = 2*np.pi/len(msg.ranges)
        for i in range(0, len(msg.ranges)):
            r = msg.ranges[i]
            x = r*np.cos(msg.ranges)
            y = r*np.sin(msg.ranges)
            self.points.append(Point(x, y, r))
        # self.l = msg.ranges[len(msg.ranges)/4] # 90 degree
        # self.b = msg.ranges[len(msg.ranges)/4*2] # 180 degree
        # self.r = msg.ranges[len(msg.ranges)/4*3] # 270 degree


    # test function:
    # reading the back,left,right distances of the Lidar
    def read_laser(self):
        
        while not self.ctrl_c:
            filtered = filterHorizon(self.points)
            if(len(filtered) > 1):
                self.aroundObject(filtered, self.setSpeed)

    def shutdownhook(self):
        self.ctrl_c = True
    


        
if __name__ == '__main__':
    rospy.init_node('rosbot_test',anonymous=False) 
    rosbot = Rosbot() 
    try:
        rosbot.read_laser()
        
    except rospy.ROSInterruptException:
        traceback.print_exc()
        pass
    