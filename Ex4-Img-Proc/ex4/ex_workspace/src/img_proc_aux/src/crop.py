#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageCropper:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.crop)
        self.pub = rospy.Publisher("image_cropped", Image, queue_size=10)
    
    def crop(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Cropping image from [startY:endY, startX, endX]
        cropped_img = cv_img[100:250, 100:250]
        #Converting image from cv format to ROS msg
        ros_cropped = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
        #publishing cropped image
        self.pub.publish(ros_cropped)



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_cropper", anonymous=True)
    img_crop = ImageCropper()
    rospy.spin()