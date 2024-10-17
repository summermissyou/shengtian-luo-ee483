#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorFilter:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.red_filter)
        self.pub = rospy.Publisher("bgr_mask", Image, queue_size=10)
    
    def red_filter(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Finds all pixels within the given range. In this scenario its red
        mask = cv2.inRange(cv_img, (0,0,0),(50,50,255))
        #Converting image from cv format to ROS msg using mono8 since it is in grayscale
        ros_red = self.bridge.cv2_to_imgmsg(mask, "mono8")
        #publishing cropped image
        self.pub.publish(ros_red)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("bgr_mask", anonymous=True)
    img_filter = ColorFilter()
    rospy.spin()