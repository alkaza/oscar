#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

theta = 90
speed = 1000
servo = 0.54

speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

def getRange(data,theta):
	# field of view	
	fov_lidar = data.angle_max - data.angle_min
	fov_car = math.pi
	fov_diff = (fov_lidar - fov_car)/2

	index = int((math.radians(theta) + fov_diff)/data.angle_increment)

	index_min = int(fov_diff/data.angle_increment)
	index_max = int((fov_car + fov_diff)/data.angle_increment)	
	if(index < index_min): index = index_min
	if(index > index_max): index = index_max
	
	dist = data.ranges[index]

	if(dist < data.range_min): return data.range_min
	elif(dist > data.range_max): return data.range_max
	else: return dist

def obstFinder(data, theta):
	a = getRange(data, theta)
	if(a < 0.6): speed = 0
	else: speed = 1000
	return speed

def callback(data):
	speed = obstFinder(data, theta)
	print("speed: " + str(speed))
	speed_pub.publish(speed)
	servo_pub.publish(servo)
	
if __name__ == '__main__':
	print("Basic Obstacle Avoidance")
	rospy.init_node('basic_obst_avoid',anonymous = True)
	rospy.Subscriber("scan_front",LaserScan,callback)
	rospy.spin()
