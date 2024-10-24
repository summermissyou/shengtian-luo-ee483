#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import Vector2D

class CoordTransf:
    def __init__(self):
        # Instatiate the converter class once by using a class member

        self.pub = rospy.Publisher("world_coord", Vector2D, queue_size=10)
        self.pub2 = rospy.Publisher("sensor_coord", Vector2D, queue_size=10)
    
    def coord_transf(self,x,y):
        '''
        Robot is at position (10,20) facing 45 degrees to the left of the x-axis

        The sensor is 1 meter forward of the center of the robot facing 90 degrees to the right relative to robot x-axis

        What is the position for the following obstacles from the sensor in world coordinates?
            1. (5,5)
            2. (10,25)
            3. (2,6)
        '''
        #Object coordinates from sensor
        
        
        sensor_pos = np.array([[x],[y],[1]]) # Recall that we need the 1 in the bottom since transformation matrices are 3x3

        #Transformation matix for sensor to robot frame - sensor2robot
        #Constructing as steps
        theta = np.radians(-90)
        sensor2robotrotation = np.round(np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]), decimals=2) #Sensor to robot rotation vector: Theta value for sensor position is -90 degrees compared to robot coordinates
        sensor2robotposition = np.array([[1],[0]]) #Sensor to robot position vector: 1 meter to the right
        sensor2robot = np.vstack((np.hstack((sensor2robotrotation,sensor2robotposition)), [0,0,1])) #Sensor to robot transform matrix: rrs and rps in top row with 0 and 1 on bottom row

        # OR constructing as a whole T matrix
        sensor2robot = np.round(np.array([[np.cos(theta),-np.sin(theta), 1],[np.sin(theta),np.cos(theta),0],[0,0,1]]), decimals=2)

        # Multiply transformation matrix times sensor position: return 3D vector with positions of object with respect to robot frame
        robot_pos = sensor2robot@sensor_pos 

        msg_robot = Vector2D()
        msg_robot.x = robot_pos[0]
        msg_robot.y = robot_pos[1]
        self.pub2.publish(msg_robot)


        #Transformation matix for robot to world frame - robot2world
        #Constructing as steps
        theta = np.radians(45)
        robot2worldrotation = np.round(np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]), decimals=2) #Robot to world rotation vector: Theta value for robot position is 45 degrees compared to world coordinates
        robot2worldposition = np.array([[10],[20]]) #Robot to world position vector: 10 meters to the right, 20 meters forward
        robot2world = np.vstack((np.hstack((robot2worldrotation, robot2worldposition)), [0,0,1])) #Robot to world transform matrix: wrr and wpr in top row with 0 and 1 on bottom row
        
        # OR constructing as a whole T matrix
        robot2world = np.round(np.array([[np.cos(theta),-np.sin(theta),10],[np.sin(theta),np.cos(theta),20],[0,0,1]]), decimals=2) 
        # Multiply transformation matrix times robot position: return 3D vector with positions of object with respect to world frame
        world_pos = robot2world@robot_pos 
        msg_world = Vector2D()
        msg_world.x = world_pos[0]
        msg_world.y = world_pos[1]
        self.pub.publish(msg_world)

        #Sensor object coords in world coords
        # wts = wtr @ rts #Matrix multiplication to find sensor to world transformation matrix
        # world_coords = wts @ np.vstack((sensor_coords.T, np.ones(sensor_coords.shape[0])))
        # msg = Vector2D()
        # i=0
        # while i<world_coords.shape[0]:
        #     msg.x = world_coords[:,i][0]
        #     msg.y = world_coords[:,i][1]
        #     self.pub.publish(msg)
        #     i+=1






if __name__=="__main__":
    # initialize our node and create a publisher as normal
    rospy.init_node("coord_transf_ex", anonymous=True)
    ct = CoordTransf()
    rate = rospy.Rate(1) #creates the rate object that wil be used to control tohe frequency of the loop to 1Hz
    sensor_coords = np.array([[5,5],[10,25],[2,6]])
    while not rospy.is_shutdown(): #initiates that the loop will run until rospy shutdown signal is recieved
        for i in range(sensor_coords.shape[0]):
            x = sensor_coords[i][0]
            y = sensor_coords[i][1]
            ct.coord_transf(x,y)
            rate.sleep()
    rospy.spin()