import rospy

from sensor_msgs.msg import LaserScan


class Rosbot():
    def __init__(self):
        
        self.rosbot_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.l = 0.0 # left (The Robot's Right)
        self.b = 0.0 # back (The Robot's Front)
        self.r = 0.0 # right (The Robot's Left)
        self.ctrl_c = False
        
        self.rate = rospy.Rate(10) #10 hz
        
        rospy.on_shutdown(self.shutdownhook)
        
    def scan_callback(self,msg):
        
        ###############################################################
        #### NOTE : The RP Lidar is able to read valus > 0.135(m) #####
        ###############################################################
        # len.msg.ranges = 720
        
        self.l = msg.ranges[len(msg.ranges)/4] # 90 degree
        self.b = msg.ranges[len(msg.ranges)/4*2] # 180 degree
        self.r = msg.ranges[len(msg.ranges)/4*3] # 270 degree
        
    def read_laser(self):
        
        while not self.ctrl_c:
            
            if self.l > 5:
                self.l = 5
                
            if self.b >5:
                self.b = 5
                
            if self.r > 5:
                self.r =5
            
            print ("b = " + str(self.b) + " r = " + str(self.r) + " l = " + str(self.l))

    def shutdownhook(self):
        self.ctrl_c = True
    


        
if __name__ == '__main__':
    rospy.init_node('rosbot_test',anonymous=True) 
    rosbot = Rosbot() 
    try:
        rosbot.read_laser()
        
    except rospy.ROSInterruptException:
        traceback.print_exc()
        pass
    