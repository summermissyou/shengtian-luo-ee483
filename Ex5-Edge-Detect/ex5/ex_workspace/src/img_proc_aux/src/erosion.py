#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Erode:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.red_mask)
        self.pub = rospy.Publisher("erode", Image, queue_size=10)
    
    def red_mask(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Convert image from BGR to HSV
        hsv_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        #Finds all pixels within the given range. In this scenario its red
        mask1 = cv2.inRange(hsv_image, (170,100,20),(180,255,255))
        mask2 = cv2.inRange(hsv_image, (0,100,100),(10,255,255))
        mask = cv2.bitwise_or(mask1, mask2)
        #Separates the red parts from original image
        red_part = cv2.bitwise_and(cv_img,cv_img,mask=mask)
        #Erode mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        # image_erode = cv2.erode(red_part, kernel)
        image_erode = cv2.erode(mask, kernel)
        #Converting image from cv format to ROS msg using bgr
        # ros_erode = self.bridge.cv2_to_imgmsg(image_erode, "bgr8")
        ros_erode = self.bridge.cv2_to_imgmsg(image_erode, "mono8")

        #publishing cropped image
        self.pub.publish(ros_erode)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("erode", anonymous=True)
    img_filter = Erode()
    rospy.spin()