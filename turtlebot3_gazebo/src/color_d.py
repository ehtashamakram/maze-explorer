#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import imutils

def image_callback(msg):
    global front,right,back,left,task_3
    bridge = cv_bridge.CvBridge()
    frame = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([136, 87, 111])
    upper_red = np.array([180, 255, 255])

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

        if area >1000 and task_3 == "true":
            print "red color"

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

        if area >1000 and task_3 == "true":
            print "green color"


rospy.init_node('follower')
image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
rospy.spin()