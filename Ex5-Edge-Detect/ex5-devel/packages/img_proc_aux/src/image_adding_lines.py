#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineTesting:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.bridge = CvBridge()
        rospy.Subscriber("image", Image, self.adding_lines)
        self.pub = rospy.Publisher("image_with_lines", Image, queue_size=10)

    
    def adding_lines(self, msg):
        lines = [[[20,30,640,400]]] #vector(x0,y0,x1,y1)- example output from houghlinesp. It will be on the top left of the image
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")        
        image_with_lines = self.output_lines(cv_img, lines) #Maps the line to the image
        ros_lines = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
        self.pub.publish(ros_lines)
    

    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_adding_lines", anonymous=True)
    img_filter = LineTesting()
    rospy.spin()