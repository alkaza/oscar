#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

desired_trajectory = 1.2
theta = 50
prev_error = 0.0
servo = 0.5
speed = 500
kp = 14.0 * 2
kd = 30 * 2

speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

def valMap(val, in_min, in_max, out_min, out_max):
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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
	
def control(error):
	global prev_error
	angle = (kp * error + kd * (prev_error - error))
	if (angle > 100): angle = 100
	elif (angle < -100): angle = -100
	prev_error = error
	servo = valMap(angle, -100.0, 100.0, 0.0, 1.0)
	return servo

def callback(data):
	error = distFinder(data, theta)
	print("error: " + str(error))
	servo = control(error)
	print("servo: " + str(servo))

	speed_pub.publish(speed)
	servo_pub.publish(servo)

	
if __name__ == '__main__':
	print("Basic Path Planning")
	rospy.init_node('basic_path_plan',anonymous = True)
	rospy.Subscriber("scan_front",LaserScan,callback)
	rospy.spin()
