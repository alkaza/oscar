#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('oscar')
import sys
import rospy
import cv2
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

prev_error = 0.0
servo = 0.5
speed = 1000
kp = 1.0
kd = 0

speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
image_pub = rospy.Publisher("lane_image", Image, queue_size=1)

bridge = CvBridge()

def valMap(val, in_min, in_max, out_min, out_max):
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# PD control - not necessary	
def control(error):
	global prev_error
	error2 = (abs(error)**0.5)*error / 13
	angle = (kp * error2 + kd * (-prev_error + error))
	if (angle > 25): angle = 25
	elif (angle < -25): angle = -25
	prev_error = error
	servo = valMap(angle, -25.0, 25.0, 0.0, 1.0) # change offset limits and test

	if servo <= 0.8 and servo >= 0.2 : return servo
	elif servo > 0.8 : return 0.8
	else : return 0.2


	return servo

import lane_track.imageop as iop
import matplotlib.pyplot as plt
import lane_track.pipeline as pipe
#plt.ion()
#fig=plt.figure()
#ax = fig.add_subplot(111)
#myobj = plt.imshow()

def lane_detect(cv_image):
	#gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('gray image', gray_image)
	#color_binary, combined_binary = iop.color_gradient_transform(cv_image)
	#	YOUR CODE
	#print('hi')
	#fig.canvas.draw()
	error = 0.0

	#cv2.imshow("Lane Line Detection", cv_image)
	#fig = plt.gcf()
	#fig.show()
	#plt.imshow(combined_binary, cmap='gray')
	#plt.show(block=True)
	#plt.show()	
	#plt.pause(0.0001)
		
	#cv2.imshow(combined_binary, cmap='gray')
	""" yecho """
	# Original Image
	#cv2.imshow('Original', cv_image)
	
	# Gray Image
	#gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('Gray', gray_image)

	# Gradient
#	colored_binary, combined_binary = iop.color_gradient_transform(cv_image)
#	cv2.imshow('Colored Binary', colored_binary)
#	cv2.imshow('Combined Binary', combined_binary)

	# Pipeline
#	pipe.advanced_lane_detection_pipeline(cv_image)	

	# OpenCV callback

	# Chaewon
	result, error, binary_warp, color_warp = pipe.advanced_lane_detection_pipeline(cv_image)
	cv2.imshow('1', result)
#	cv2.imshow('2', binary_warp)
#	cv2.imshow('3', color_warp)
	# End Chaewon
	cv2.waitKey(1)
	
#	try:
#		image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#	except CvBridgeError as e:
#		print(e)
	
	return error #offset

def callback(data):
	try:
		cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
	except CvBridgeError as e:
		print(e)

	error = lane_detect(cv_image)
	print("error: " + str(error))
	servo = control(error)
	print("servo: " + str(servo))
	print("speed: " + str(speed))
	print()

	speed_pub.publish(speed)
	servo_pub.publish(servo)
	
if __name__ == '__main__':
	print("Lane Line Detection")
	rospy.init_node('lane_detect',anonymous = True)
	rospy.Subscriber('/zed/left/image_rect_color', Image, callback)
	rospy.spin()
