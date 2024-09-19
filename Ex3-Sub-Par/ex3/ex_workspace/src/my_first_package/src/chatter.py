#!/usr/bin/env python3
# Adapted from Prof Robinette's EECE5560 class 
import rospy
from std_msgs.msg import String

class Listener: # Defines a class for the subscriber node
	def __init__(self):# Defines the constructor for the Talker class 
		# In the constructor, you want to initialize all variables needed for your code
		# The init function is called once when you create an instance of the class
		rospy.Subscriber("chatter", String, self.callback) # Register the subscriber topic with ROS. 
						#The topic name is chatter
						#The message type is listening for is of String type
						# Once it receives a message, it will call the self.callback function
						# Whenever a new message arrives, this function will be called
										
	def callback(self, msg): # Function that will be called when a String message is received in the chatter topic
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data) # This will print the message received
			
if __name__ == '__main__':
	try:
		rospy.init_node('listener', anonymous=True) #Initiates the ROS listener node
		Listener() # Constructs Listener class
		rospy.spin() # spin() simply keeps python from exiting until this node is stopped
	except rospy.ROSInterruptException:
		pass