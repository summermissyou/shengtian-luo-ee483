#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtleMove: #class object
    
    def __init__(self): #constructor method that intializes the object
        self.pub = rospy.Publisher('turtle/turtle1/cmd_vel', Twist, queue_size=10) #publisher
        self.front=True 
    def talk(self): #defines method
        msg = Twist() #new message object we will added the turtle's movements
        if self.front: # Move ahead
            msg.linear.x = 1
        else: # Move back
            msg.linear.x = -1
            self.front = True
        rospy.loginfo(msg)
        self.pub.publish(msg) # publishes the Twist msg to the topic
        
        rate.sleep()


if __name__ == '__main__': #checks if the script is being run directly
    try:
        rospy.init_node('turtle_move', anonymous=True) #initializes ros node 'turtle_move', anonymous being set to true ensures if another node with the same name is running, a unique name will be assigned to it
        t = TurtleMove() # Creates an instance of the TurtleMove Class - At this point, it will run the class init function
        rate = rospy.Rate(1) #creates the rate object that wil be used to control tohe frequency of the loop to 1Hz
        while not rospy.is_shutdown(): #initiates that the loop will run until rospy shutdown signal is recieved
            t.talk()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

