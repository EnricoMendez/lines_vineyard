#!/usr/bin/env python3 

#Code to filter the iamge with HSV values

import cv2
import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time
import os

class ImageFilter():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  

        ############################### SUBSCRIBERS #####################################   
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object
        self.image_received = 0 #Flag to indicate that we have already received an image
        self.cv_image = 0 #This is just to create the global variable cv_image 
        self.bridge = CvBridge()

        rate = 10  #Hz
        seconds = 15

        ############ VARIABLES ################
        aux_cont = 0
        img_cont = 0
        path = '/home/oem/estancia22/vineyard_ws/src/lines_vineyard/scripts/test_images'

        #********** INIT NODE **********###  
        r = rospy.Rate(rate)  
        print("Node initialized 10hz") 
        while not rospy.is_shutdown():  
            if self.image_received:
                cv2.waitKey(1) 
                if aux_cont == (rate * seconds):
                    aux_cont = 0
                    timestr = time.strftime("%Y%m%d-%H%M%S")
                    print(cv2.imwrite(os.path.join(path , timestr+'.jpg'), self.cv_image))
                    
                    img_cont += 1
                    print(img_cont)
                else:
                    aux_cont += 1
            r.sleep()  
        cv2.destroyAllWindows()       

         
    def camera_callback(self,data):
        self.image_received=1
        try:
            #print("received ROS image")
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
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