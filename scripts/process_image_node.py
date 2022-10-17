#!/usr/bin/env python3 

#Code to process image obtained

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import time

class ImageFilter():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)  
        ###******* INIT PUBLISHERS *******###  
        self.pub=rospy.Publisher("otsuImage",Image,queue_size=10)
        self.pub2=rospy.Publisher("greenxImage",Image,queue_size=10)
        self.pub3=rospy.Publisher("blobImage",Image,queue_size=10)

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

    def blob_morph(self,img):
        contours,hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        maxContour = 0
        for contour in contours:
            print(contour)
            contourSize = cv2.contourArea(contour)
            if contourSize > maxContour:
                maxContour = contourSize
                maxContourData = contour

        # Create a mask from the largest contour
        mask = np.zeros_like(img)
        mask = mask.astype('uint8')
        mask = cv2.fillPoly(mask,[maxContourData],1)

        return mask



    def erosion(self,nombre):
        kernel = np.array([[1,1,1],[1,1,1],[1,1,1]])
        imagen_erosion = cv2.morphologyEx(nombre,cv2.MORPH_ERODE,kernel)
        return imagen_erosion  
        
    def greenIndex(self,image):
            x,y,z = image.shape
            gexg = np.zeros([x,y])
            r = image[:,:,0]
            g = image[:,:,1]
            b = image[:,:,2]

            x,y = r.shape

            for i in range(x-1):
                for j in range (y-1):
                    if g[i,j] < r[i,j] or g[i,j] < b[i,j]:
                        gexg[i,j] = 0
                    else:
                        gexg[i,j] = 2*g[i,j]-r[i,j]-b[i,j]


            gexg = gexg.astype('uint8')
            image_message = self.bridge.cv2_to_imgmsg(gexg, encoding="passthrough")
            self.pub2.publish(image_message)
            return gexg
            
    def otsu(self,image):
        imageGray = self.greenIndex(image)
        blur = cv2.GaussianBlur(imageGray,(5,5),0)
        ret,otsu = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        image_message = self.bridge.cv2_to_imgmsg(otsu, encoding="passthrough")
        self.pub.publish(image_message)
        return otsu

         
    def camera_callback(self,data):
        self.image_received=1
        try:
            print("received ROS image, I will convert it to opencv")
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            inicio = time.time()
            image = self.otsu(self.cv_image)
            image = self.blob_morph(image)
            fin = time.time()
            print(fin-inicio)
            image_message = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.pub3.publish(image_message)

            #cv2.imshow("img",image)
            #cv2.waitKey(0)
            
        except CvBridgeError as e:
            print(e) 

    def cleanup(self):  
        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  
if __name__ == "__main__":  
    rospy.init_node('opencv_otsu', anonymous=True)
    ImageFilter() 