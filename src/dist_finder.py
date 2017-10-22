#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from control.msg import pid_input

desired_trajectory = 0
vel = 30

pub = rospy.Publisher('control/error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here
	index=(theta+45)*4
	distance=data.ranges[index]
	return distance

def callback(data):
	theta = 45;
	a = getRange(data,theta)
	b = getRange(data,0)
	swing = math.radians(theta)
	c=a*math.cos(swing)-b
	d=a*math.sin(swing)
	beta=math.atan2(c,d)
	A=b*math.cos(beta)

	a1 = getRange(data,180-theta)
	b1 = getRange(data,180)

	c1=a1*math.cos(swing)-b
	d1=a1*math.sin(swing)
	beta1=math.atan2(c1,d1)
	A1=b1*math.cos(beta1)

	error=(A-A1-desired_trajectory)*2


			
	## Your code goes here


2

	## END

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("catvehicle/front_laser_points",LaserScan,callback)
	rospy.spin()
