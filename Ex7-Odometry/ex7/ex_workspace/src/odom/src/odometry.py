#!/usr/bin/env python3

import rospy
from odom_aux.msg import DistWheel, Pose2D
import math

class PoseCalculator:
    def __init__(self):
        # Initialize node
        rospy.init_node('pose_calculator')

        # Set up subscriber to receive wheel distance updates
        self.dist_sub = rospy.Subscriber('dist_wheel', DistWheel, self.calculate_pose_callback)

        # Set up publisher to publish the robot's pose
        self.pose_pub = rospy.Publisher('pose', Pose2D, queue_size=10)

        # Initialize robot's pose
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0  # Orientation in radians

        # Baseline (half the distance between the wheels)
        self.L = 0.05  # meters

    def calculate_pose_callback(self, msg):
        # Extract incremental distances traveled by each wheel
        d_left = msg.dist_wheel_left
        d_right = msg.dist_wheel_right
       
        # Calculate the change in orientation (theta)
        delta_theta = (d_right - d_left) / (2 * self.L)

        # Calculate the average distance moved
        delta_s = (d_right + d_left) / 2
        rospy.loginfo(f"delta_s: {delta_s}")

        # Apply updated equations for delta_x and delta_y using mid-point orientation
        delta_x = delta_s * math.cos(self.pose.theta + delta_theta / 2)
        delta_y = delta_s * math.sin(self.pose.theta + delta_theta / 2)

    # Update pose
        self.pose.x += delta_x
        self.pose.y += delta_y
        self.pose.theta += delta_theta  # Update orientation

        a=math.cos(self.pose.theta + delta_theta / 2)
        b=math.sin(self.pose.theta + delta_theta / 2)

        rospy.loginfo(f"cos is: {a},sin is: {b}")
        
        # Publish the updated pose
        self.pose_pub.publish(self.pose)
        rospy.loginfo(f"Updated Pose - x: {self.pose.x}, y: {self.pose.y}, theta: {self.pose.theta}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PoseCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass
