#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan



roll = pitch = yaw = 0.0
target = -90
kp=0.5

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw

left = 0
right = 0
back = 0
front = 0
right_front=0
right_back=0
left_back = 0
left_front = 0
back_right = 0
back_left = 0
front_right = 0
front_left = 0



def scan_callback(msg):

	global left
	global right
	global front
	global back
	global right_front
	global right_back
	global left_front
	global left_back
	global back_right
	global back_left
	front = msg.ranges[0]
	#print len(msg.ranges)
	front_right = msg.ranges[330]	
	front_left = msg.ranges[40]
	right = msg.ranges[len(msg.ranges)*3/4]				
	right_back = msg.ranges[len(msg.ranges)*118/160] 	
	right_front = msg.ranges[len(msg.ranges)*122/160] 
	back = msg.ranges[len(msg.ranges)/2]
	back_right = msg.ranges[len(msg.ranges)*82/160] 
	back_left = msg.ranges[len(msg.ranges)*78/160]
	left = msg.ranges[len(msg.ranges)/4]
	left_front = msg.ranges[len(msg.ranges)*38/160]
	left_back = msg.ranges[len(msg.ranges)*42/160]


def stop():
	twist.angular.z = 0
	twist.linear.x = 0
	pub.publish(twist)

def set_orientation():
	print yaw
	if yaw > 0 and yaw < 1:
		print "1st"
		while yaw >0 :
			twist.angular.z = 0-yaw
			pub.publish(twist)
		stop()
	if yaw > 1 and yaw < 2:
		#print "2nd"
		while 1:
			#print "2nd"
			twist.angular.z = 1.57-yaw
			pub.publish(twist)
			if yaw < 1.59 and yaw > 1.57:
				break
	if yaw <3.1 and yaw >2:
		print "3rd"
		while yaw<3.12:
			twist.angular.z = 3.12-yaw
			pub.publish(twist)
		stop()

	if yaw < 0 and yaw > -1:
		print "4th"
		while yaw < -0.1 :
			twist.angular.z = 0.1 - yaw
			pub.publish(twist)
		stop()

	if yaw < -1 and yaw > -2:
		#print "2nd"
		while 1:
			#print "2nd"
			twist.angular.z = 1.57-yaw
			pub.publish(twist)
			if yaw > -1.59 and yaw < -1.57:
				break
	if yaw >-3.1 and yaw < -2:
		print "3rd"
		while yaw > -3.12:
			twist.angular.z = -3.12-yaw
			pub.publish(twist)
		stop()

angular_speed = 0.5
def turn():
    twist.angular.z = angular_speed
    t0  = rospy.Time.now().to_sec()
    angle_travelled = 0

    while ( angle_travelled < math.pi/1.6 ):
    	print"turning"
        pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        angle_travelled = angular_speed*(t1-t0)
    stop()
    rospy.sleep(0.5)

def turn_right():
	#angular_speed = 0.5
    twist.angular.z = -angular_speed
    t0  = rospy.Time.now().to_sec()
    angle_travelled = 0

    while ( angle_travelled < math.pi/1.6 ):
        pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        angle_travelled = angular_speed*(t1-t0)
    stop()
    rospy.sleep(0.5)


def turn_180():
	#angular_speed = 0.5
    twist.angular.z = -angular_speed
    t0  = rospy.Time.now().to_sec()
    angle_travelled = 0

    while ( angle_travelled < math.pi/1 ):
        pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
        angle_travelled = angular_speed*(t1-t0)
    stop()
    rospy.sleep(0.5)

rospy.init_node('turtlebot3')
scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.Rate(10)
twist =Twist()


def h():
	if left > 1:
		print left, type(left), "1st"
	else:
		print "i don't know"
while not rospy.is_shutdown():
	h()
	rospy.sleep(0.01)