#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleMove:
    def __init__(self):
        rospy.init_node('turtle_move_square', anonymous=True)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz frequency
        self.pose = Pose()
        self.start_pose = None
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)  

        self.twist = Twist()
        self.square_size = 2.0  # edge length 
        self.is_moving_forward = True

    def update_pose(self, data):
        self.pose = data

        # node down original point
        if self.start_pose is None:
            self.start_pose = data

    def move_forward(self, distance):
        start_x = self.pose.x
        start_y = self.pose.y
        while not rospy.is_shutdown():
            self.twist.linear.x = 1.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)

            # 计算乌龟移动的距离
            distance_moved = math.sqrt((self.pose.x - start_x) ** 2 + (self.pose.y - start_y) ** 2)
            if distance_moved >= distance:
                break

            self.rate.sleep()

        self.stop()

    def turn(self, angle):
        start_angle = self.pose.theta
        relative_angle = 0.0
        angular_speed = 0.785  # angular velocity

        while relative_angle < angle:
            self.twist.linear.x = 0.0
            self.twist.angular.z = angular_speed
            self.pub.publish(self.twist)

            # calculate turing angle
            relative_angle = abs(self.pose.theta - start_angle)
            self.rate.sleep()

        self.stop()

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)
        rospy.sleep(1)  # stop

    def move_square(self):
        for _ in range(4):
            self.move_forward(self.square_size)  # moving
            self.turn(1.57)  # turn 90 degree
        rospy.loginfo("stop")
        
        self.correct_final_position()

    def correct_final_position(self):
        
        error_x = self.start_pose.x - self.pose.x
        error_y = self.start_pose.y - self.pose.y

        
        self.move_forward(abs(error_x))

        
        self.turn(1.57)  
        self.move_forward(abs(error_y))

if __name__ == '__main__':
    try:
        turtle_move = TurtleMove()
        turtle_move.move_square()  
    except rospy.ROSInterruptException:
        pass
