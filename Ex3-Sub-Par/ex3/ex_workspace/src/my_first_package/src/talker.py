#!/usr/bin/env python3
# Adapted from Prof Robinette's EECE5560 class 
import rospy
from std_msgs.msg import String

class Talker: # Defines a class for the publisher node.
	def __init__(self): # Defines the constructor for the Talker class 
		# In the constructor, you want to initialize all variables needed for your code
		# The init function is called once when you create an instance of the class
		self.pub = rospy.Publisher('chatter', String, queue_size = 10)
		# self.pub registers a publisher with ROS - That's your variable name - it creates the publisher
		# 'chatter' is the name of the topic
		# String is the type of message being publish in the topic. String is a message type from the class std_msg.msg.String
		# queue_size limists the amount of queued messages
	def talker(self): # This will be the function with the main functionalities of our class Talker
		hello_str = 'Hello World: ' + str(rospy.get_time())
		rospy.loginfo(hello_str) # Logs the message into the roslog. It  will also print the hello_str variable in the terminal
		self.pub.publish(hello_str) # Publishing the message hello_str 
		
if __name__ == '__main__':
	try:
		rospy.init_node('talker', anonymous=True) # Registers node with ROS as node name "talker"
		t = Talker() #creating an instance of the Talker class
		rate = rospy.Rate(10) # Runs once a 0.1secs or 10Hz
		while not rospy.is_shutdown(): #It will run until ros master gets shutdown 
			t.talker() # Call the talk function to publish the 'chatter' topic
			rate.sleep() # Waits for one second

	except rospy.ROSInterruptException:
		pass