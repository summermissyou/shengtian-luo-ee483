#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
from duckietown_msgs.msg import Vector2D

class CoordTransf:
    def __init__(self):
        
        self.pub_robot = rospy.Publisher("robot_coord", Vector2D, queue_size=10)
        self.pub_world = rospy.Publisher("world_coord", Vector2D, queue_size=10)
        
       
        rospy.Subscriber("sensor_coord", Vector2D, self.sensor_callback)

    def sensor_callback(self, msg):
      # get the position from sensor
        sensor_x = msg.x
        sensor_y = msg.y

       
        self.coord_transf(sensor_x, sensor_y)


    def coord_transf(self, x, y):
        '''
        Your robot is in position (5, 3) facing 45 degrees left of the y-axis (135 degrees relative to x-axis) in world coordinates.
        It has a sensor pointed exactly behind the robot (180 degrees from forward/positive x-axis on the robot) and placed 1 m behind the center of the robot.

        Robot position (5,3) facing 135 degrees in world coordinates
        Sensor position (-1,0) facing 180 degrees in robot coordinates

        The sensor returns obstacles in the following locations (in the sensor coordinate frame):

            (7,2)
            (16,-1)
            (-7,6)
            (-8,-13)

        '''
        
        sensor_pos = np.array([[x], [y], [1]])

        #Transformaton matix for sensor to robot frame - sensor2robot
        #Constructing as steps
        theta0=np.radians(180)
        sensor2robotrotation = np.round(np.array([[np.cos(theta0),-np.sin(theta0)],[np.sin(theta0),np.cos(theta0)]]), decimals=2) #Sensor to robot rotation vector: Theta value for sensor position is 180 degrees compared to robot coordinates
        sensor2robotposition = np.array([[-1],[0]]) #Sensor to robot is (-1,0)
        sensor2robot = np.vstack((np.hstack((sensor2robotrotation,sensor2robotposition)), [0,0,1])) #Sensor to robot transform matrix: rrs and rps in top row with 0 and 1 on bottom row
        
         # OR constructing as a whole T matrix
        sensor2robot = np.round(np.array([[np.cos(theta0),-np.sin(theta0), -1],[np.sin(theta0),np.cos(theta0),0],[0,0,1]]), decimals=2)
        
         # Multiply transformation matrix times sensor position: return 3D vector with positions of object with respect to robot frame
        robot_pos = sensor2robot@sensor_pos 
        
        msg_robot = Vector2D()
        msg_robot.x = robot_pos[0]
        msg_robot.y = robot_pos[1]
        self.pub_robot.publish(msg_robot)

        #Transformation matix for robot to world frame - robot2world
        #Constructing as steps
        theta1 = np.radians(135)
        robot2worldrotation = np.round(np.array([[np.cos(theta1),-np.sin(theta1)],[np.sin(theta1),np.cos(theta1)]]), decimals=3) #Robot to world rotation vector: Theta value for robot position is 135 degrees compared to world coordinates
        robot2worldposition = np.array([[5],[3]]) #Robot to world position vector: 5 meters to the right, 3 meters forward
        robot2world = np.vstack((np.hstack((robot2worldrotation, robot2worldposition)), [0,0,1])) #Robot to world transform matrix: wrr and wpr in top row with 0 and 1 on bottom row

        # OR constructing as a whole T matrix
        robot2world = np.round(np.array([[np.cos(theta1),-np.sin(theta1),5],[np.sin(theta1),np.cos(theta1),3],[0,0,1]]), decimals=3) 
       
        
        # Multiply transformation matrix times robot position: return 3D vector with positions of object with respect to world frame
        world_pos = robot2world@robot_pos 
        print("world_pos:\n", world_pos)
        msg_world = Vector2D()
        msg_world.x = world_pos[0]
        msg_world.y = world_pos[1]
        self.pub_world.publish(msg_world)


if __name__ == "__main__":
    rospy.init_node("coord_transf_node", anonymous=True)
    ct = CoordTransf()
    rospy.spin()
