#!/usr/bin/env python3 
from os import stat
import rospy 
import numpy as np
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.pyplot import ion, show, plot
from datetime import datetime

class validation_class:
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        
        ############ INIT PUBLISHERS ############ 
        status_pub = rospy.Publisher('validation', String, queue_size=10)

        ############ SUSCRIBER ############
        rospy.Subscriber("front/scan", LaserScan, self.lidar_cb) 

        ############ CONSTANTS ############
        
        ############ VARIABLES ############
        self.error = np.array(0)
        self.start = datetime.now()
        self.time = np.array(0)
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
            self.error = np.append(self.error,center_ref)
            moment = datetime.now() - self.start
            self.time = np.append(self.time,moment.seconds)
            # plt.plot(self.time,self.error)
            # plt.xlabel('Time (s)')
            # plt.ylabel('Error (m)')
            # plt.show()
            status_pub.publish(status)
    def lidar_cb(self,data):
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
    def cleanup(self):
        plt.plot(self.time,self.error)
        plt.xlabel('Time (s)')
        plt.ylabel('Error (m)')
        plt.show()
        pass

if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    validation_class()