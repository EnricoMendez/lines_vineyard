#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
import math

# Initial variables

bridgeObject = CvBridge()                                           # Create cv_bridge object
cvImage = 0                                                         # Create image variable
imageFlag = 0                                                       # Inidicates if an images has been recived
bridge = CvBridge()
image = 0
pub=rospy.Publisher("newImage",Image,queue_size=10)

def node_callback(data):
    global bridgeObject
    global cvImage
    global imageFlag
    global bridge
    try:
        imageFlag = 1

        cvImage = bridgeObject.imgmsg_to_cv2(data)
    except:
        return
def originalImage(data):
    global bridgeObject
    global image
    global imageFlag
    global bridge
    try:
        imageFlag = 1

        image = bridgeObject.imgmsg_to_cv2(data)
    except:
        return

def imageProcess():
    #cvImage = cv2.imread('dave.jpg')
    edges = cv2.Canny(cvImage,50,150,apertureSize = 3)
    cv2.imshow('edges',edges)
    lines = cv2.HoughLines(edges,1,np.pi/180,200) 
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(image, pt1, pt2, (0,255,255), 3)
            print('line',i)


    image_message = bridge.cv2_to_imgmsg(cvImage, encoding="passthrough")
    pub.publish(image_message)  
    cv2.imshow("SegmentedImage",image)
    cv2.waitKey(1)



if __name__ == '__main__' :
    rospy.init_node('imageDetectionNode', anonymous=True)           # Start node
    sub = rospy.Subscriber("/filteredImage",Image,node_callback)    # Start suscriber
    sub = rospy.Subscriber("/usb_cam/image_raw",Image,originalImage)    # Start suscriber
    while not rospy.is_shutdown():
        if not imageFlag:
            rospy.sleep(1)
            continue
        imageProcess()
        rospy.sleep(1)
    