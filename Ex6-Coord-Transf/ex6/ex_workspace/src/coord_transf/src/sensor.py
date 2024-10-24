#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import Vector2D

class Sensor:
    def __init__(self):
        # Instatiate the converter class once by using a class member
        self.pub = rospy.Publisher("sensor_coord", Vector2D, queue_size=10)
    
    def sensor(self,x,y):
        '''
        Publishes position of objects detected by the sensor
            - (7,2)
            - (16,-1)
            - (-7,6)
            - (-8,-13)
        '''
             
        msg = Vector2D()
        msg.x = x
        msg.y = y
        rospy.loginfo("Publishing ("+str(x)+"," +str(y)+") position")
        self.pub.publish(msg)

if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("coord_transf_ex", anonymous=True)
    sensor = Sensor()
    rate = rospy.Rate(1) #creates the rate object that wil be used to control tohe frequency of the loop to 1Hz
    sensor_coords = np.array([[7,2],[16,-1],[-7,6],[-8,-13]])   
    # print(sensor_coords.shape)
    while not rospy.is_shutdown(): #initiates that the loop will run until rospy shutdown signal is recieved
        i=0
        for i in range(sensor_coords.shape[0]):
            x = sensor_coords[i][0]
            y = sensor_coords[i][1]
            sensor.sensor(x,y)
            rospy.sleep(5)
            i+=1
        
    rospy.spin()