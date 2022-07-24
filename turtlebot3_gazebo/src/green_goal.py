#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
import numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image
import imutils
from sensor_msgs.msg import LaserScan

#initializing the values of roll pitch yaw
roll = pitch = yaw = 0.0

#This function decode the information frorm Odometry to get position and orientation 
def get_rotation (msg):
    global roll, pitch, yaw, a, b
    a = msg.pose.pose.position.x
    b = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    #convert quatenian to euler angles
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

#initializing the values of robot 4 sides
left = 0
right = 0
back = 0
front = 0

#initializing color variables
red = 0
green = 0

#This function decode the information frorm Camera to detect the colors
def image_callback(msg):
    global red, green
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)

    kernel = np.ones((5,5),'int')
    dilated = cv2.dilate(mask,kernel)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask=mask)
    ret,thrshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    contours,hier = cv2.findContours(thrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        #Contour area is taken
        area = cv2.contourArea(cnt)
        if area >1000:
            print "red color"
            red = 1
        else:
        	red = 0

    # define range of green color in HSV
    lower_green = np.array([25, 52, 72])
    upper_green = np.array([102, 255, 255])

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5,5),'int')
    dilated = cv2.dilate(mask,kernel)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask=mask)
    ret,thrshed = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),3,255,cv2.THRESH_BINARY)
    contours,hier = cv2.findContours(thrshed,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        #Contour area is taken
        area = cv2.contourArea(cnt)

        if area >1000:
            print "green color"
            green = 1

    #cv2.imshow('camera', frame)

    #--- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cv2.destroyAllWindows()

#This function gets the information from lidar and calculate the 4 sides distances from obstacles
def scan_callback(msg):
	global left
	global right
	global front
	global back
	front = msg.ranges[0]
	right = msg.ranges[len(msg.ranges)*3/4]				
	back = msg.ranges[len(msg.ranges)/2]
	left = msg.ranges[len(msg.ranges)/4]


#Publishes the 0 value to /cmd_vel topic to stop the robot
def stop():
	print "stop"
	twist.angular.z = 0
	twist.linear.x = 0
	pub.publish(twist)


#turn clockwise 90 degree
def turn():
	print "turn"
	n=0
	if yaw < -2.2:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw<-1.5 and yaw>-1.6:
				break
		stop()
	if yaw > 2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw<-1.5 and yaw>-1.6:
				break
		stop()

	if yaw > 1 and yaw < 2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw>3.04 and yaw<3.14:
				break
		stop()
	if yaw<-1 and yaw>-2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw > 0 and yaw < 0.2:
				break
		stop()
	if yaw > -1 and yaw < 1 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw>1.5 and yaw<1.6:
				break
		stop()

#turn anticlockwise 90 degree
def turn_a():
	print "turn_a"
	n = 0
	if yaw>1 and yaw<2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = -0.5
			pub.publish(twist)
			if yaw > 0 and yaw < 0.2:
				break
		stop()
	if yaw<-2.5:
		n = 1
		while 1:
			twist.angular.z = -0.5
			pub.publish(twist)
			if yaw>1.5 and yaw<1.6:
				break
		stop()
	if yaw > 2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = -0.5
			pub.publish(twist)
			if yaw>1.5 and yaw<1.6:
				break
		stop()
	if yaw < -1 and yaw > -2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = -0.5
			pub.publish(twist)
			if yaw>3.04 and yaw<3.14:
				break
		stop()
	if yaw > -1 and yaw < 1 and n ==0:
		n = 1
		while 1:
			twist.angular.z = -0.5
			pub.publish(twist)
			if yaw < -1.5 and yaw > -1.6:
				break
		stop()

#turn 180 degree
def turn_180():
	print "turn_180"
	n=0
	if yaw < -2.2:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw< 0.1 and yaw>0:
				break
		stop()
	if yaw > 2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw<0.1 and yaw>0:
				break
		stop()

	if yaw > 1 and yaw < 2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw<-1.57 and yaw>-1.6:
				break
		stop()
	if yaw<-1 and yaw>-2.2 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw > 1.5 and yaw < 1.6:
				break
		stop()
	if yaw > -1 and yaw < 1 and n ==0:
		n = 1
		while 1:
			twist.angular.z = 0.5
			pub.publish(twist)
			if yaw>3.05 and yaw<3.14:
				break
		stop()

#Always keep orientation parallel to the walls
def orientation():
	if yaw > 0 and yaw < 1:
		twist.angular.z = 0.01-yaw
	if yaw > 1 and yaw < 2:
		twist.angular.z = 1.57-yaw
	if yaw <3.1 and yaw >2:
		twist.angular.z = 3.12-yaw
	if yaw < -0.1 and yaw > -1:
		twist.angular.z = -0.01 - yaw
	if yaw < -1 and yaw > -2:
		twist.angular.z = -1.57-yaw
	if yaw >-3.1 and yaw < -2:
		twist.angular.z = -3.12-yaw

#Move straight i distance
def move(i):
	print "move"
	c = a
	d = b
	if yaw > -1 and yaw < 1 :
		while 1:
			dif = c - a
			twist.linear.x = 0.3
			orientation()
			pub.publish(twist)
			if dif < 0:
				dif = - dif
			if dif >i:
				break
			if red == 1:
				break
	if yaw > 2.5:
		while 1:
			dif = c - a
			twist.linear.x = 0.3
			orientation()
			pub.publish(twist)
			if dif < 0:
				dif = - dif
			if dif >i:
				break
			if red == 1:
				break
	if yaw < -2.5:
		while 1:
			dif = c - a
			twist.linear.x = 0.3
			orientation()
			pub.publish(twist)
			if dif < 0:
				dif = - dif
			if dif >i:
				break
			if red == 1:
				break
	if yaw < 2.5 and yaw > 1:
		while 1:
			dif = d - b
			twist.linear.x = 0.3
			orientation()
			pub.publish(twist)
			if dif < 0:
				dif = - dif
			if dif >i:
				break
			if red == 1:
				break
	if yaw < -1 and yaw > -2.5:
		while 1:
			dif = d - b
			twist.linear.x = 0.3
			orientation()
			pub.publish(twist)
			if dif < 0:
				dif = - dif
			if dif >i:
				break
			if red == 1:
				break

def extra():
	o = 0
	if red == 1:
		if right >1 and back > 1 and o == 0:
			print "called"
			stop()
			turn_a()
			#move(0.9)
			stop()
			o = 1
		if right < 1 and back > 1 and o == 0:
			print " 2nd called"
			stop()
			turn_180()
			#move(0.9)
			stop()
			o = 1

h = 0
#This is the main logic behind every condition
def main():
	global a, b, red, green, front, right, left, h
	if front > 0.6:
		twist.linear.x = 0.3
		orientation()
		pub.publish(twist)
		print" moving forward"

	if front > 0.6 and red == 1:
		n = 0
		stop()
		if left > 0.9 and right > 0.9 and n == 0:
			n = 1
			turn()
			extra()
			move(0.7)
			stop()
		if left < 0.9 and right > 0.9 and n == 0:
			n = 1
			turn_a()
			move(0.7)
			stop()
		if left < 0.9 and right < 0.9 and n == 0:
			n = 1
			turn_180()
			move(0.7)
			stop()
		if left > 0.9 and right > 0.9 and n == 0:
			n = 1
			turn()
			extra()
			move(0.7)
			stop()

	if front > 0.6 and left > 1.2 and h > 4:
		stop()
		move(0.4)
		stop()
		turn()
		extra()
		move(1)
		stop()
		if left > 1:
			turn()
			extra()
			stop()
			move(0.9)
			stop()

	if front <= 0.6:
		stop()
		if left > 0.9 and right < 0.9:
			turn()
			extra()
			h = h + 1
		if left < 0.9 and right > 0.9:
			turn_a()
			h = h +1
		if left < 0.9 and right < 0.9:
			turn_180()
			h = h + 1
		if left > 0.9 and right > 0.9:
			turn()
			extra()
			h = h + 1

	if green == 1:
		stop()
		move(0.4)
		stop()
		while 1:
			print "Goal Reached"
	# else:
	# 	print "something went wrong"


rospy.init_node('turtlebot3')
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
image_sub = rospy.Subscriber('/up_front_d435/camera/image', Image, image_callback)
rospy.Rate(10)
twist =Twist()


j=0
while not rospy.is_shutdown():
	while j<100000:
		j = j+1
		print "getting ready"
	main()
	rospy.sleep(0.01)