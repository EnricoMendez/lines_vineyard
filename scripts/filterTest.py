#!/usr/bin/env python3
from logging import shutdown
from tkinter import Frame
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np 
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

bridge_object = CvBridge() # create the cv_bridge object
image_received = 0 #Flag to indicate that we have already received an image
cv_image = 0 #This is just to create the global variable cv_image 
frame = 0
bridge = CvBridge()



max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
    
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
    
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
    
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
    
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
    
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)
    
#parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
#parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
#args = parser.parse_args()

#cap = cv2.VideoCapture(0)

cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
def obtenerImagen(data):
    global bridge_object
    global cv_image
    global image_received
    global frame
    image_received=1

    # We select bgr8 because its the OpenCV encoding by default
    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

    #frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    frame = cv_image
def main():   
    global image_received
    global frame

    while not rospy.is_shutdown():
        #ret, frame = cap.read()
        if not image_received:
            rospy.sleep(1)
            continue 
        if frame is None:
            break
                    
        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
        
        
        cv2.imshow(window_capture_name, frame)
        cv2.imshow(window_detection_name, frame_threshold)
        
        key = cv2.waitKey(30)
        if key == ord('q') or key == 27:
            break
        rospy.sleep(1)
    #cap.release()
    # Closes all the frames
    cv2.destroyAllWindows()
if __name__ == '__main__':
    pub=rospy.Publisher("segmented_image",Image,queue_size=10)
    rospy.init_node('filterCalibration', anonymous=True)
    image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,obtenerImagen)
    main()