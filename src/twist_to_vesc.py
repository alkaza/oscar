#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

speed_to_erpm_gain = 4614
speed_to_erpm_offset = 0.0
steering_angle_to_servo_gain = -1.2135
steering_angle_to_servo_offset = 0.5304
wheelbase = 0.25

speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

def callback(twist):
	curr_speed = twist.linear.x
	curr_steering_angle = 0.0
	curr_angular_vel = twist.angular.x

	speed = curr_speed * speed_to_erpm_gain + speed_to_erpm_offset
	
	curr_steering_angle = math.atan2(curr_angular_vel * wheelbase, curr_speed) 
	servo = curr_steering_angle * steering_angle_to_servo_gain + steering_angle_to_servo_offset

#	if (speed > 5000.0): speed = 5000.0
#	elif (speed < -5000.0): speed = -5000.0

#	if (servo > 1.0): servo = 1.0
#	elif (servo < 0.0): servo = 0.0

	speed_pub.publish(speed)
	servo_pub.publish(servo)
	
if __name__ == '__main__':
	print("Twist to VESC")
	rospy.init_node('twist_to_vesc', anonymous = True)
	rospy.Subscriber('cmd_vel', Twist, callback)
	rospy.spin()
