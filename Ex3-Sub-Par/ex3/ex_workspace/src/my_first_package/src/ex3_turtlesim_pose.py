#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class TurtleDistanceNode:
    def __init__(self):
        # ROS publishers and subscribers
        self.pose_sub = rospy.Subscriber('turtle1/pose', Pose, self.pose_callback)
        self.dist_pub = rospy.Publisher('turtle1/total_distance', Float32, queue_size=10)
        
        # Distance variables
        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        
        # Rate of publishing
        self.rate = rospy.Rate(1)  # 1Hz
        
    def pose_callback(self, data):
        """Callback function to calculate the distance and publish it"""
        # If previous position is not set, initialize it
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = data.x
            self.prev_y = data.y
            return

        # Calculate the Euclidean distance between current and previous position
        distance = ((data.x - self.prev_x) ** 2 + (data.y - self.prev_y) ** 2) ** 0.5
        
        # Update total distance traversed
        self.total_distance += distance

        # Update previous position
        self.prev_x = data.x
        self.prev_y = data.y

        # Check if the distance should be in meters or feet
        self.publish_distance()

    def publish_distance(self):
        """Publishes the total distance in meters or feet based on a ROS parameter"""
        # Get the distance unit parameter (default is meters)
        unit = rospy.get_param('/distance_unit', 'meters')
        distance_to_publish = self.total_distance
        
        # Convert distance to feet if required
        if unit == 'feet':
            distance_to_publish *= 3.28084  # 1 meter = 3.28084 feet
        
        # Publish the distance to the new topic
        self.dist_pub.publish(distance_to_publish)

        # Log the distance and units
        rospy.loginfo(f"Total distance: {distance_to_publish:.2f} {unit}")
        
    def run(self):
        """Runs the main loop of the node"""
        rospy.loginfo("Turtle Distance Node started...")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the node
        rospy.init_node('turtle_distance_node', anonymous=True)
        
        # Create an instance of the TurtleDistanceNode class
        turtle_distance_node = TurtleDistanceNode()
        
        # Run the node
        turtle_distance_node.run()
        
    except rospy.ROSInterruptException:
        pass