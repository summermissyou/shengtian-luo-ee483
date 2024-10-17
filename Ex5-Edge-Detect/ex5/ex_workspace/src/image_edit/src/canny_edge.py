#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

class ImageEdges:
    def __init__(self):

        # Instantiate the CvBridge object once as a class member
        self.bridge = CvBridge()

        # Subscribe to the cropped and filter image topic
        cropped_image_sub = Subscriber("image_cropped", Image)
        white_mask_sub = Subscriber("image_white", Image)
        
        # Use message_filters to synchronize the topics
        ats = ApproximateTimeSynchronizer([cropped_image_sub, white_mask_sub], queue_size=10,slop=0.1)
        ats.registerCallback(self.image_callback)

        # Publisher for the edge-detected image
        self.pub_edges = rospy.Publisher("image_edges", Image, queue_size=10)
        self.pub_combined=rospy.Publisher("combined_image", Image, queue_size=10)
        
        self.pub_lines = rospy.Publisher("image_line_white", Image, queue_size=10)

    

    def image_callback(self, cropped_msg, mask_msg):
       
        # Convert the incoming ROS image message to OpenCV format
        cropped_img = self.bridge.imgmsg_to_cv2(cropped_msg, "bgr8")
        white_mask = self.bridge.imgmsg_to_cv2(mask_msg, "mono8")
       
        # Convert the image to grayscale
        gray_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and improve edge detection
        blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)


       # Perform Canny edge detection on the blurred image
        edges = cv2.Canny(blurred_img, 50, 150, apertureSize=3)
       
       

        white_mask_resized = cv2.resize(white_mask, (edges.shape[1], edges.shape[0]))

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        mask_eroded = cv2.erode(white_mask_resized, kernel, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, kernel, iterations=2)
        

         # Combine the edges with the white mask using bitwise_and
        combined_result = cv2.bitwise_and(edges, mask_dilated)

        #Hough transform for combined image
        lines=cv2.HoughLinesP(combined_result, 1, np.pi/180, 3, minLineLength=1, maxLineGap=5)

        #Maps the line to the image
        image_with_lines = self.output_lines(cropped_img, lines)

        #Converting image from cv format to ROS msg
        ros_edges = self.bridge.cv2_to_imgmsg(edges, "mono8")
        ros_combined = self.bridge.cv2_to_imgmsg(combined_result, "mono8")
        ros_lines = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")

        #publish the  image
        self.pub_lines.publish(ros_lines)
        self.pub_combined.publish(ros_combined)
        self.pub_edges.publish(ros_edges)

        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (255, 0, 0), 2, cv2.LINE_AA)
        return output


if __name__ == "__main__":
    # Initialize the ROS node
        rospy.init_node("image_edges", anonymous=True)

    # Create an instance of the ImageEdges class
        ImageEdges()

    # Keep the node running
        rospy.spin()
