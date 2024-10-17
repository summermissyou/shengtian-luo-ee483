#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageFlipper:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.flipper_cb)
        self.pub = rospy.Publisher("flipped", Image, queue_size=10)
    
    def flipper_cb(self, msg):
        # convert to a ROS image using the bridge
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # flip along the horizontal axis using an OpenCV function
        cv_flipped = cv2.flip(cv_img, 0)
        
        # convert new image to ROS to send
        ros_flipped = self.bridge.cv2_to_imgmsg(cv_flipped, "bgr8")
        
        # publish flipped image
        self.pub.publish(ros_flipped)
        

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_flipper", anonymous=True)
    img_flip = ImageFlipper()
    rospy.spin()
