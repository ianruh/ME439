"""
title:lab 1 - ME 439 Intro to robotics
"""

#------------imports----------------
import serial
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)
import time

# Motor Demo
for spd in range(0,481,1):
	motors.motor1.setSpeed(spd)
	time.sleep(0.01)
	
for spd in range(480,-481,-1):
	motors.motor1.setSpeed(spd)
	time.sleep(0.01)
	
for spd in range(-480,1,1): # stops before it gets to 1/100
	motors.motor1.setSpeed(spd)
	time.sleep(0.01)
	
print( 'Final Speed: {0}'.format(spd) )


