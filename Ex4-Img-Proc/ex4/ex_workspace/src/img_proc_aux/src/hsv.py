#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Hsv:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.hsv_image)
        self.pub = rospy.Publisher("hsv_image", Image, queue_size=10)
    
    def hsv_image(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Convert image from BGR to HSV
        hsv_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        #Convert back image from HSV to BGR
        bgr_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

        #Converting image from cv format to ROS msg using bgr8
        ros_hsv = self.bridge.cv2_to_imgmsg(bgr_image, "bgr8")
        #publishing cropped image
        self.pub.publish(ros_hsv)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("hsv", anonymous=True)
    img_filter = Hsv()
    rospy.spin()