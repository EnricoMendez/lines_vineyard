#!/usr/bin/env python3 

#Code to filter the image with the otsu threshold

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import os

class ImageFilter():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub=rospy.Publisher("otsuImage",Image,queue_size=10)

        ############################### SUBSCRIBERS #####################################   
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        self.image_sub = rospy.Subscriber("/realsense/color/image_raw",Image,self.camera_callback) 
        
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object
        self.image_received = 0 #Flag to indicate that we have already received an image
        self.cv_image = 0 #This is just to create the global variable cv_image 
        self.bridge = CvBridge()

        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        print("Node initialized 10hz") 
        while not rospy.is_shutdown():  
            if self.image_received:
                cv2.waitKey(1) 
                r.sleep()  
        cv2.destroyAllWindows()        


    def image_processing(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        x,y = otsu.shape
        maxval = 0
        otsu[0,639]
        for i in range(y):
            value = np.sum(otsu[:,i])
            if value > maxval:
                maxval = value
                index = i
        
        cv2.line(image,(index,0),(index,x),(255,0,0),9)

        return image

         
    def camera_callback(self,data):
        self.image_received=1
        try:
            print("received ROS image, I will convert it to opencv")
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            inicio = time.time()
            image = self.image_processing(self.cv_image)
            fin = time.time()
            print(fin-inicio)
            image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.pub.publish(image_message)
            
        except CvBridgeError as e:
            print(e) 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('tie_method', anonymous=True)
    ImageFilter() 