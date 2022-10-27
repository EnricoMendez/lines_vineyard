from os import stat
import rospy 

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
        
        aux = 0
        for i in range(5):
            aux = data.ranges[i]
        for i in range(5):
            aux = data.ranges[-i]
        self.right_dist = aux / 10 

        aux = 0
        for i in range(5):
            aux = data.ranges[lenght/2 + i]
        for i in range(5):
            aux = data.ranges[lenght/2 - i]
        self.left_dist = aux / 10 

    
    def cleanup(self):
        pass

if __name__ == "__main__":
    rospy.init_node("navigation", anonymous=True)
    validation_class()