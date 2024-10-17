#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GaussianBlur:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.blur)
        self.pub = rospy.Publisher("image_gauss", Image, queue_size=10)
    
    def blur(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Apply a gaussian blur to the image
        gauss_img = cv2.GaussianBlur(cv_img, (3,3),0)
        #Converting image from cv format to ROS msg using mono8 since it is in grayscale
        ros_gauss = self.bridge.cv2_to_imgmsg(gauss_img, "bgr8")
        #publishing cropped image
        self.pub.publish(ros_gauss)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("kernel_op", anonymous=True)
    GaussianBlur()
    rospy.spin()