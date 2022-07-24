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







def stop():
    print "stop"
    twist.angular.z = 0
    twist.linear.x = 0
    pub.publish(twist)


def set_orientation():
    if yaw > 0.1 and yaw < 1:
        while yaw >0.07 :
            twist.angular.z = 0.07-yaw
            pub.publish(twist)
        stop()
    if yaw > 1 and yaw < 2:
        while 1:
            twist.angular.z = 1.57-yaw
            pub.publish(twist)
            if yaw == 1.57:
                break
    if yaw <3.1 and yaw >2:
        while yaw<3.1:
            twist.angular.z = 3.1-yaw
            pub.publish(twist)
        stop()

angular_speed = 0.5

def turn():
    if yaw > 1 and yaw < 2.2:
        print "2 ",yaw
        while 1:
            twist.angular.z = 0.5
            pub.publish(twist)
            if yaw>3.04 and yaw<3.14:
                break
        stop()
        print "calling main"
        main()


    if yaw > 2.2 and yaw < 3.14:
        print "3 ",yaw
        while 1:
            twist.angular.z = 0.5
            pub.publish(twist)
            if yaw<-1.5 and yaw>-1.6:
                break
        stop()
        print "calling main"
        main()

    if yaw<-1 and yaw>-2.2:
        print "4 ",yaw
        while 1:
            twist.angular.z = 0.5
            pub.publish(twist)
            if yaw > 0 and yaw < 0.2:
                break
        stop()
        print "calling main"
        main()

    if yaw > -1 and yaw < 1:
        print "1st ",yaw
        while 1:
            twist.angular.z = 0.5
            pub.publish(twist)
            if yaw>1.5 and yaw<1.6:
                break
        stop()
        print "calling main"
        main()






def turn_a():

    if yaw>1 and yaw<2.2:
        print "4 ",yaw
        while 1:
            twist.angular.z = -0.5
            pub.publish(twist)
            if yaw > 0 and yaw < 0.2:
                break
        stop()
        print "calling main"
        main()


    if yaw > 2.2 and yaw < 3.14:
        print "3 ",yaw
        while 1:
            twist.angular.z = -0.5
            pub.publish(twist)
            if yaw>1.5 and yaw<1.6:
                break
        stop()
        print "calling main"
        main()


    if yaw < -1 and yaw > -2.2:
        print "2 ",yaw
        while 1:
            twist.angular.z = -0.5
            pub.publish(twist)
            if yaw>3.04 and yaw<3.14:
                break
        stop()
        print "calling main"
        main()



    if yaw > -1 and yaw < 1:
        print "1st ",yaw
        while 1:
            twist.angular.z = -0.5
            pub.publish(twist)
            if yaw < -1.5 and yaw > -1.6:
                break
        stop()
        print "calling main"
        main()






#def turn():


#def turn_right():





def main():

    
rospy.init_node('turtlebot3')
#scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.Rate(10)
twist =Twist()


# if __name__ == '__main__':
#   main()
n=0
while not rospy.is_shutdown():
    main()
    rospy.sleep(0.01)