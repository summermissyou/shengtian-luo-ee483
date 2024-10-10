#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorMask:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.red_mask)
        self.pub = rospy.Publisher("red_mask", Image, queue_size=10)
    
    def red_mask(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Convert image from BGR to HSV
        hsv_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        #Finds all pixels within the given range. In this scenario its red
        mask1 = cv2.inRange(hsv_image, (0,97,181),(180,255,255))
        mask2 = cv2.inRange(hsv_image, (88,133,0),(104,248,184))
        mask = cv2.bitwise_or(mask1, mask2)

        #Separates the red parts from original image
        blue_red_part = cv2.bitwise_and(cv_img,cv_img,mask=mask)
        #Converting image from cv format to ROS msg using 
        ros_red = self.bridge.cv2_to_imgmsg(blue_red_part, "bgr8")
        #publishing cropped image
        self.pub.publish(ros_red)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("red_mask", anonymous=True)
    img_filter = ColorMask()
    rospy.spin()