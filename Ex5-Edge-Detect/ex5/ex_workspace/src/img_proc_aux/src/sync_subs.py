#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
##IMPORT MESSAGE FILTERS
from message_filters import ApproximateTimeSynchronizer, Subscriber

class Sync:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        crop = Subscriber("/image_cropped", Image)
        red = Subscriber("/red_filter", Image)
        ats = ApproximateTimeSynchronizer([crop,red],queue_size=5,slop=0.1)
        ats.registerCallback(self.callback)
        self.pub = rospy.Publisher("/image_combined", Image, queue_size = 10)
    
    def callback(self,msg1,msg2): #msg1 is the cropped image, while msg2 is the red_filter
        #Pulling image from ROS msg to cv format
        cv_cropped = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
        cv_red = self.bridge.imgmsg_to_cv2(msg2, "mono8")
        cv_red_cropped = cv_red[100:250, 100:250]
        combined = cv2.bitwise_and(cv_cropped,cv_cropped,mask=cv_red_cropped)
        ros_combined = self.bridge.cv2_to_imgmsg(combined, "bgr8")
        self.pub.publish(ros_combined)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("sync", anonymous=True)
    img_crop = Sync()
    rospy.spin()