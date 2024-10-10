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
        self.pub_white = rospy.Publisher("image_white", Image, queue_size=10)
        self.pub_dila =rospy.Publisher("mask_dilation", Image,queue_size=10)
    
    def crop(self, msg):

        #Pulling image from ROS msg to cv format
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        #Cropping image from [startY:endY, startX, endX]
        # Get the image dimensions
        height, width, _ = cv_img.shape
        startY=height//2
        
        # Define the crop: take the bottom half of the image
       
        cropped_img = cv_img[startY:height, 0:width]

        # Convert cropped image to HSV color space
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
          
        # Define range for white color and create a mask
        mask1 = cv2.inRange(hsv_img, (0, 97, 181), (180, 255, 255))
        mask2 = cv2.inRange(hsv_img, (88,133,0),(104,248,184))
        mask = cv2.bitwise_or(mask1, mask2)

         # Bitwise-AND the mask and cropped image to keep the white parts
        output_img= cv2.bitwise_and(cv_img, cv_img, mask=mask)

        #Dilate mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        image_dilate = cv2.dilate(mask, kernel)
    
        #Converting image from cv format to ROS msg
        ros_cropped = self.bridge.cv2_to_imgmsg(cropped_img, "bgr8")
        ros_white_filtered = self.bridge.cv2_to_imgmsg(output_img, "bgr8")
        ros_dilate=self.bridge.cv2_to_imgmsg(image_dilate,"mono8")



         

        #publishing cropped image
        self.pub.publish(ros_cropped)
        self.pub_white.publish(ros_white_filtered)
        self.pub_dila.publish(ros_dilate)
       



if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("image_cropper", anonymous=True)
    img_crop = ImageCropper()
    rospy.spin()