#!/usr/bin/env python3

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge_object = CvBridge() # create the cv_bridge object
image_received = 0 #Flag to indicate that we have already received an image
cv_image = 0 #This is just to create the global variable cv_image 
bridge = CvBridge()

pub=rospy.Publisher("filteredImage",Image,queue_size=10)

def erosion(nombre):
    kernel = np.array([[1,1,1],[1,1,1],[1,1,1]])
    imagen_erosion = cv2.morphologyEx(nombre,cv2.MORPH_ERODE,kernel)
    return imagen_erosion

def filtroColor(image):
    #Once we read the image we need to change the color space to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #Hsv limits are defined
    #here is where you define the range of the color youre looking for
    #each value of the vector corresponds to the H,S & V values respectively
    # min_color = np.array([38,11,146])
    # max_color = np.array([162,38,178])

    #min_color = np.array([0,0,0])
    #max_color = np.array([180,63,152])
    
    #min_color = np.array([0,0,51])
    #max_color = np.array([103,50,126])

    min_color = np.array([0,0,0])
    max_color = np.array([180,99,181])
    

    #This is the actual color detection 
    #Here we will create a mask that contains only the colors defined in your limits
    #This mask has only one dimension, so its black and white }
    colorMask = cv2.inRange(hsv, min_color, max_color)
    colorMask = erosion(colorMask)
    #We use the mask with the original image to get the colored post-processed image
    result = cv2.bitwise_and(image,image, mask= colorMask)
    return colorMask

def mainfunc():
    global image
    image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,camera_callback)

    r = rospy.Rate(10) #10Hz 
    while not rospy.is_shutdown(): 
        if image_received:
            cv2.waitKey(1)
            r.sleep() 
    cv2.destroyAllWindows()

def camera_callback(data):
    global bridge_object
    global cv_image
    global image_received
    global image
    image_received=1
    try:
        print("received ROS image, I will convert it to opencv")
        # We select bgr8 because its the OpenCV encoding by default
        cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        image = filtroColor(cv_image)
        
        image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
        pub.publish(image_message)
        
    except CvBridgeError as e:
        print(e)
       

if __name__ == '__main__':
    rospy.init_node('opencv_example1', anonymous=True)
    mainfunc()
