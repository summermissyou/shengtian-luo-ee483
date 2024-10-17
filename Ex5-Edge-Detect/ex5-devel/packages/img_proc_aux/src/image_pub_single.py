#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospkg

if __name__=="__main__":

    # initialize our node and create a publisher as normal
    rospy.init_node("image_publisher", anonymous=True)
    pub = rospy.Publisher("image", Image, queue_size=10)

    # we need to instatiate the class that does the CV-ROS conversion
    bridge = CvBridge()

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("img_proc_aux")

    # get all images and store in a list
    ros_imgs = list()
    for i in range(1):
        filename = pkg_path + "/sample_images/image" + str(i) + ".png"
        #read the image file into an OpenCV image
        cv_img = cv2.imread(filename)
        # convert to a ROS sensor_msgs/Image and store for later use
        ros_imgs.append(bridge.cv2_to_imgmsg(cv_img, "bgr8"))

    # publish ten times over a second
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        for img in ros_imgs:
            t = rospy.get_time()
            # publish each image for 5 sec
            while not rospy.is_shutdown() and rospy.get_time() - t < 5:
                pub.publish(img)
                r.sleep()