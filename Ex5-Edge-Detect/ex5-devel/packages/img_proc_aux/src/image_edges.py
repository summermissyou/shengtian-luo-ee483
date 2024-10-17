#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageEdges:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.edges)
        self.pub = rospy.Publisher("image_edges", Image, queue_size=10)
    
    def edges(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        
        #Using sobel to create outlines of image
        sobel_img = cv2.Sobel(cv_img, cv2.CV_8U, 1, 0)

        # Convert to absolute value and scale to 8-bit
        # sobel_img = np.uint8(np.absolute(sobel_img))

        sobel_img = cv2.cvtColor(sobel_img, cv2.COLOR_BGR2GRAY)
        #Converting image from cv format to ROS msg using mono8 since it is in grayscale
        ros_sobel = self.bridge.cv2_to_imgmsg(sobel_img, "mono8")
        #publishing cropped image
        self.pub.publish(ros_sobel)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_edges", anonymous=True)
    ImageEdges()
    rospy.spin()