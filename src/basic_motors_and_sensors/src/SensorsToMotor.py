#!/usr/bin/env python

import rospy
# Import "serial" to get data from the AlaMode
import serial   
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32, Float32
from mobrob_util.msg import ME439SensorsRaw
import PID_controler as robot_pid

# Set up callable Publishers and messages
pub_wheel_speed_desired_left = rospy.Publisher('/wheel_speed_desired_left',Float32,queue_size=1)
pub_wheel_speed_desired_right = rospy.Publisher('/wheel_speed_desired_right',Float32,queue_size=1)

## get params 
init_speed = rospy.get_param('init_speed')
max_motor_command = rospy.get_param('max_motor_command')
K_p = rospy.get_param('K_p')
K_d = rospy.get_param('K_d')

def sensors_to_wheel_speed():
    rospy.init_node('SensorsToMotor',anonymous=False)
    
    all_sensors = rospy.Subscriber('/sensors_raw',ME439SensorsRaw,sensors_to_motor_command)

    # prevent the node from exiting
    rospy.spin()
    

def sensors_to_motor_command(msg_in):
    
    # unpack the message
    R_s = msg_in.a0 # right sensor
    M_s = msg_in.a1 # middle sensor
    L_s = msg_in.a2 # left sensor
    
    error = 0
    no_line = 0
    # ERRORS: White:0 Black:1
        
    if(R_s > 100. and M_s > 100. and L_s < 100.): # 0 0 1
        error = 2
    elif(R_s > 100. and M_s < 100. and L_s < 100.):# 0 1 1
        error = 1
    elif(R_s > 100. and M_s < 100. and L_s > 100.):# 0 1 0
        error = 0
    elif(R_s < 100. and M_s < 100. and L_s > 100.):# 1 1 0
        error = -1
    elif(R_s < 100. and M_s > 100. and L_s > 100.):# 1 0 0
        error = -2  
    elif((R_s > 100. and M_s > 100. and L_s > 100.) or # 1 1 1 or 0 0 0
         (R_s < 100. and M_s < 100. and L_s < 100.)):
        no_line = 1
    # PID control
    PID  = robot_pid.robot_PID(error,K_p,K_d)
    PID.PIDcontrol()
    
    #checking sensors
    print(str(error)+'  '+str(R_s)+' '+str(M_s)+' '+str(L_s)) 
    
    # if all sensors detects black or white stop 
    ##################
    # we will need to change this for going around obstacle
    ###############
    if(no_line == 1):
        wheel_speed_desired_left = 0.0
        wheel_speed_desired_right = 0.0
    elif(no_line == 0): # if one of the sensors detects the line
        wheel_speed_desired_left = init_speed - PID.PID_val # set left speed
        wheel_speed_desired_right = init_speed + PID.PID_val #set right speed
        
        #limit the speed less than max_speed        
        if (wheel_speed_desired_left > max_motor_command):
            wheel_speed_desired_left = max_motor_command
        if (wheel_speed_desired_right > max_motor_command):
            wheel_speed_desired_right = max_motor_command
    
    # pack and publish left
    wheel_speed_desired_left_msg = Float32()
    wheel_speed_desired_left_msg.data = wheel_speed_desired_left
    pub_wheel_speed_desired_left.publish(wheel_speed_desired_left_msg)
    
    # pack and publish right
    wheel_speed_desired_right_msg = Float32()
    wheel_speed_desired_right_msg.data = wheel_speed_desired_right
    pub_wheel_speed_desired_right.publish(wheel_speed_desired_right_msg)
      
    
# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        sensors_to_wheel_speed()
    except rospy.ROSInterruptException: 
        pass
