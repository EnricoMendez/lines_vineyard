#!/usr/bin/env python3 

#Code to filter the iamge with HSV values

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImageFilter():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub=rospy.Publisher("filteredImage",Image,queue_size=10)

        ############################### SUBSCRIBERS #####################################   
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        
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

    def erosion(self,nombre):
        kernel = np.array([[1,1,1],[1,1,1],[1,1,1]])
        imagen_erosion = cv2.morphologyEx(nombre,cv2.MORPH_ERODE,kernel)
        return imagen_erosion  

    def filtroColor(self,image):
        #Once we read the image we need to change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Hsv limits are defined
        #here is where you define the range of the color youre looking for
        #each value of the vector corresponds to the H,S & V values respectively
        # min_color = np.array([38,11,146])
        # max_color = np.array([162,38,178])

        min_color = np.array([0,0,0])
        max_color = np.array([180,63,152])
        
        #min_color = np.array([0,0,51])
        #max_color = np.array([103,50,126])

        #This is the actual color detection 
        #Here we will create a mask that contains only the colors defined in your limits
        #This mask has only one dimension, so its black and white }
        colorMask = cv2.inRange(hsv, min_color, max_color)
        colorMask = self.erosion(colorMask)
        #We use the mask with the original image to get the colored post-processed image
        result = cv2.bitwise_and(image,image, mask= colorMask)
        return colorMask

         
    def camera_callback(self,data):
        self.image_received=1
        try:
            print("received ROS image, I will convert it to opencv")
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            image = self.filtroColor(self.cv_image)
            
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
    rospy.init_node('opencv_filter', anonymous=True)
    ImageFilter() 