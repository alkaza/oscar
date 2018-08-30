#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import curses
#import signal
#TIMEOUT = 0.1 # number of seconds your want for timeout
speed = 0.0;
turn = 0.5;

# def interrupted(signum, frame):
#     "called when read times out"
#     global speed
#     speed = 0
#     global turn
#     turn = 0
#     stdscr.addstr(2, 20, "Stop")
#     stdscr.addstr(2, 25, '%.2f' % speed)
#     stdscr.addstr(3, 20, "Stop")
#     stdscr.addstr(3, 25, '%.2f' % turn)
# signal.signal(signal.SIGALRM, interrupted)

# def input():
#     try:
#             foo = stdscr.getch()
#             return foo
#     except:
#             # timeout
#             return

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
rospy.init_node('keyboard_control', anonymous=True)
speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
servo_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

# set alarm
#signal.alarm(TIMEOUT)
#s = input()
# disable the alarm after success
#signal.alarm(0)
#print 'You typed', s

stdscr.refresh()

key = ''
while key != ord('q'):
#	signal.setitimer(signal.ITIMER_REAL,0.05)
#	key = input()
	key = stdscr.getch()
	stdscr.refresh()
#	signal.alarm(0)
	stdscr.addstr(2, 20, "Move around with arrow keys")
	stdscr.addstr(3, 20, "Press 'delete' to stop and 'q' to quit")
	if key == curses.KEY_UP: 
		speed = speed + 500.0;
		stdscr.addstr(5, 20, "Up  ")
		stdscr.addstr(5, 25, '%.2f' % speed)
		stdscr.addstr(8, 20, "    ")
	elif key == curses.KEY_DOWN:
		speed = speed - 500.0; 
		stdscr.addstr(5, 20, "Down")
		stdscr.addstr(5, 25, '%.2f' % speed)
		stdscr.addstr(8, 20, "    ")
	if key == curses.KEY_LEFT:
		turn = turn - 0.05; 
		stdscr.addstr(6, 20, "Left")
		stdscr.addstr(6, 25, '%.2f' % turn)
		stdscr.addstr(8, 20, "    ")
	elif key == curses.KEY_RIGHT:
		turn = turn + 0.05; 
		stdscr.addstr(6, 20, "Rght")
		stdscr.addstr(6, 25, '%.2f' % turn)
		stdscr.addstr(8, 20, "    ")
	if key == curses.KEY_DC:
		speed = 0.0;
		turn = 0.5;
		stdscr.addstr(5, 25, '%.2f' % speed)
		stdscr.addstr(6, 25, '%.2f' % turn)
		stdscr.addstr(8, 20, "Stop")

	speed_pub.publish(speed)
	servo_pub.publish(turn)

curses.endwin()
