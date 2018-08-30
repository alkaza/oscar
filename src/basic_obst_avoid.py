#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

desired_trajectory = 1.2
theta = 50
prev_error = 0.0
speed = 2000
servo = 0.5
kp = 15.0
kd = 0.09

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

def distFinder(data, theta):
	a = getRange(data,theta)
	b = getRange(data,0)
	swing = math.radians(theta)
	
	alpha = math.atan2(a * math.cos(swing) - b, a * math.sin(swing))
	AB = b * math.cos(alpha)
	AC = 1
	CD = AB + AC * math.sin(alpha)
	error = CD - desired_trajectory
	return error

def obstFinder(data, theta):
	a = getRange(data,90)
	if(a < 0.5): speed = 0
	else: speed = 2000
	return speed

def control(error):
	global prev_error
	servo = (kp * error + kd * (prev_error - error))/100
	prev_error = error
	return servo

def callback(data):
	speed = obstFinder(data, theta)
	print("speed: " + str(speed))
#	error = distFinder(data, theta)
#	print("error: " + str(error))
#	servo = control(error)
#	print("servo: " + str(servo))

	speed_pub.publish(speed)
	servo_pub.publish(servo)
	
if __name__ == '__main__':
	print("Basic Obstacle Avoidance")
	rospy.init_node('basic_obst_avoid',anonymous = True)
	rospy.Subscriber("scan_front",LaserScan,callback)
	rospy.spin()
