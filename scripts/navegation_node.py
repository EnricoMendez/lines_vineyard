#!/usr/bin/env python3 

#Code to filter the image with the otsu threshold

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
import os

class ImageFilter():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub2=rospy.Publisher("tie_method",Image,queue_size=10)
        self.pub=rospy.Publisher("otsuImage",Image,queue_size=10)
        self.pub_vel = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        ############################### SUBSCRIBERS #####################################   
        #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.camera_callback) 
        self.image_sub = rospy.Subscriber("/realsense/color/image_raw",Image,self.camera_callback) 
        self.vel_msg = Twist()
        ############ CONSTANTS ################  
        self.bridge_object = CvBridge() # create the cv_bridge object
        self.image_received = 0 #Flag to indicate that we have already received an image
        self.cv_image = 0 #This is just to create the global variable cv_image 
        self.bridge = CvBridge()
        self.vel_msg = Twist()      
        self.p = -0.01             #Control gain
        self.vel_msg.linear.x = 0.5   #Linear velocity
        self.proportion_criterion = 1.5
        self.correction_gain = 0.1  #Angular     

        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        print("Node initialized 10hz") 
        while not rospy.is_shutdown():  
            if self.image_received:
                cv2.waitKey(1) 
                r.sleep()  
        cv2.destroyAllWindows()        

    def navigation(self,y,idx,left,right):
        prop_left = left/right
        prop_right = right/left
        condition1 = prop_left > self.proportion_criterion
        condition2 = prop_right > self.proportion_criterion
        os.system('clear')
        if condition1 and not np.isposinf(prop_left):
            self.vel_msg.angular.z = self.correction_gain* (prop_left)
            print('Correction mode activated (turning right)')
            print('Proportion: ',str(prop_left))
            print('Velocity:')
            print(str(self.vel_msg))
        elif condition2 and not np.isposinf(prop_right):
            self.vel_msg.angular.z = -self.correction_gain * (prop_right)
            print('Correction mode activated (turning left)')
            print('Proportion: ',str(prop_right))
            print('Velocity:')
            print(str(self.vel_msg))
        else:
            deviation = idx-(y/2)
            self.vel_msg.angular.z= deviation*self.p
            print('Target: ',str(idx) ,'(out of ',y,')')
            print('Deviation: ',str(deviation))
            print('Velocity:')
            print(str(self.vel_msg))
        self.pub_vel.publish(self.vel_msg)

    def image_processing(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        image_message = self.bridge.cv2_to_imgmsg(otsu, encoding="passthrough")
        self.pub.publish(image_message)
        x,y = otsu.shape
        array = np.zeros((y,2))
        for i in range(y):
            value = np.sum(otsu[:,i])
            array[i,1] = value
            array[i,0] = i
        #array=np.flipud(np.sort(array))

        print('array: ',array)
        array = array[array[:, 1].argsort()[::-1]]
        

        index = int(np.mean(array[0:5,0]))
        left = np.sum(otsu[0:int(x/3*2),0:index])
        right = np.sum(otsu[0:int(x/3*2),index:y])
        self.navigation(y,index,left,right)
        print(array)
        print('left: ',left)
        print('right ',right)
        print('index: ',index)


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
            print('Cycle time:', fin-inicio,' seg')
            image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.pub2.publish(image_message)
            
        except CvBridgeError as e:
            print(e) 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node. 
        stop = Twist()
        self.pub_vel.publish(stop)
        print('Node killed successfully')   
        pass  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('tie_method', anonymous=True)
    ImageFilter() 