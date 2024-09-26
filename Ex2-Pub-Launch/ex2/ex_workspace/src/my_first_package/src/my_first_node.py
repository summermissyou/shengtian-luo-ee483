#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Global variables to keep track of the pose
x = 0
y = 0
theta = 0

# Pose callback function to update position and orientation
def pose_callback(pose):
    global x, y, theta
    x = pose.x
    y = pose.y
    theta = pose.theta

# Function to move the turtle forward a set distance
def move(distance, speed):
    velocity_message = Twist()
    velocity_message.linear.x = speed

    start_x = x
    start_y = y
    distance_traveled = 0

    # Move forward until the desired distance is reached
    while distance_traveled < distance:
        rospy.loginfo(f"Turtle moving. Distance traveled: {distance_traveled:.2f}")
        pub.publish(velocity_message)
        rospy.sleep(0.1)
        distance_traveled = math.sqrt((x - start_x) ** 2 + (y - start_y) ** 2)

    # Stop the turtle once the distance is reached
    velocity_message.linear.x = 0
    pub.publish(velocity_message)

# Function to rotate the turtle by a specific angle
def rotate(angle, angular_speed):
    velocity_message = Twist()
    velocity_message.angular.z = angular_speed if angle > 0 else -angular_speed

    # Calculate the target angle
    current_angle = 0
    rate = rospy.Rate(10)  # 10 Hz

    # Rotate until the desired angle is reached
    while abs(current_angle) < abs(angle):
        rospy.loginfo(f"Turtle rotating. Current angle: {current_angle:.2f}")
        pub.publish(velocity_message)
        rate.sleep()
        current_angle += angular_speed * 0.1  # 0.1 is the sleep time (rate of 10Hz)

    # Stop the rotation
    velocity_message.angular.z = 0
    pub.publish(velocity_message)

    def square_path():
     rospy.init_node('turtlesim_square', anonymous=True)

    # Subscribe to the turtle's pose topic to get the position and orientation
    rospy.Subscriber('turtle1/pose', Pose, pose_callback)

    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)

    rospy.sleep(2)  # Give time to initialize

    side_length = 2  # Define the length of one side of the square
    speed = 1  # Speed of movement
    angular_speed = math.radians(30)  # Speed of rotation in radians

    for _ in range(4):
        move(side_length, speed)  # Move forward by the side length
        rospy.sleep(1)  # Pause briefly after moving forward
        rotate(math.radians(90), angular_speed)  # Rotate 90 degrees
        rospy.sleep(1)  # Pause briefly after rotating

    rospy.loginfo("Square path completed!")

if __name__ == '__main__':
    try:
        square_path()
    except rospy.ROSInterruptException:
        pass