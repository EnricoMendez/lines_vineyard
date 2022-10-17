#!/usr/bin/env python3

#Node to detect lines from the image filtered

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
import math

class LineDetection():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  
        self.pub=rospy.Publisher("newImage",Image,queue_size=10)

        ############################### SUBSCRIBERS #####################################  
        self.filtered_img_sub = rospy.Subscriber("/filteredImage",Image,self.node_callback)    # Start suscriber
        self.raw_img_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.originalImage)   

        ############ CONSTANTS ################  
        self.bridgeObject = CvBridge()                                           # Create cv_bridge object
        self.cvImage = 0                                                         # Create image variable
        self.imageFlag = 0                                                       # Inidicates if an images has been recived
        self.bridge = CvBridge()
        self.image = 0

        #********** INIT NODE **********###  
        r = rospy.Rate(1) #1Hz  
        print("Node initialized 1hz") 
        while not rospy.is_shutdown():  
            if not self.imageFlag:
                rospy.sleep(1)
                continue
            self.imageProcess()
            r.sleep()  

    def node_callback(self,data):
        try:
            self.imageFlag = 1

            self.cvImage = self.bridgeObject.imgmsg_to_cv2(data)
        except:
            return 

         
    def originalImage(self,data):
        try:
            self.imageFlag = 1

            self.image = self.bridgeObject.imgmsg_to_cv2(data)
        except:
            return

    def imageProcess(self):
    #cvImage = cv2.imread('dave.jpg')
        edges = cv2.Canny(self.cvImage,50,150,apertureSize = 3)
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
                cv2.line(self.image, pt1, pt2, (0,255,255), 3)
                print('line',i)

        image_message = self.bridge.cv2_to_imgmsg(self.cvImage, encoding="passthrough")
        self.pub.publish(image_message)  
        cv2.imshow("SegmentedImage",self.image)
        cv2.waitKey(1)

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('imageDetectionNode', anonymous=True)           # Start node
    LineDetection() 