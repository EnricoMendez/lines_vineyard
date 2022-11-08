#!/usr/bin/env python3 
from os import stat
import rospy 
import numpy as np
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class validation_class:
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        
        ############ INIT PUBLISHERS ############ 
        status_pub = rospy.Publisher('validation', String, queue_size=10)

        ############ SUSCRIBER ############
        rospy.Subscriber("front/scan", LaserScan, self.lidar_cb) 

        ############ CONSTANTS ############
        
        ############ VARIABLES ############
        self.right_dist = 0
        self.left_dist = 0
        status = ""

        
        r = rospy.Rate(10)
        print('initialized node')
        while not rospy.is_shutdown():

            os.system('clear')  
            center_ref = self.right_dist - self.left_dist
            print("Center: ",center_ref)
            if center_ref == 0:
                status = "center"
            elif center_ref < 0:
                status = "left"
            else:
                status = "right"

            print("Deviated to: ",status)  
            r.sleep()

            status_pub.publish(status)

    def lidar_cb(self,data):
        lenght = len(data.ranges)
        start = data.angle_min
        end = data.angle_max
        iterations = len(data.ranges)
        list_angles = np.linspace(start,end,iterations)*180/np.pi
        list_ranges = np.array(data.ranges,dtype=np.float64)
        
        left= list_ranges[(list_angles>=86) & (list_angles<=94)]
        right= list_ranges[(list_angles<=-86) & (list_angles>=-94)]
        
        right = np.nan_to_num(right,posinf=np.NAN) 
        left = np.nan_to_num(left,posinf=np.NAN)
        
        self.right_dist = np.nanmean(right)
        self.left_dist = np.nanmean(left)
        #aux = 0
        #for i in range(5):
        #    aux = data.ranges[i]
        #for i in range(5):
        #    aux = data.ranges[-i]
        #self.right_dist = aux / 10 

        #aux = 0
        #for i in range(5):
        #    aux = data.ranges[int(lenght/2)-1 + i]
        #for i in range(5):
        #    aux = data.ranges[int(lenght/2)-1 - i]
        #self.left_dist = aux / 10 

    
    def cleanup(self):
        pass

if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    validation_class()