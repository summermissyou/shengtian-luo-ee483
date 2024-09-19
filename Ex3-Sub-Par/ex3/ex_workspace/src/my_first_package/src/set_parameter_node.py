#!/usr/bin/env python3

import rospy

class SetParameterNode:
    def __init__(self):
        # Rate at which to toggle the parameter
        self.rate = rospy.Rate(0.2)  # 0.2 Hz, i.e., every 5 seconds

    def run(self):
        rospy.loginfo("Set Parameter Node started...")

        while not rospy.is_shutdown():
            # Check if the parameter '/distance_unit' exists
            if rospy.has_param('/distance_unit'):
                # Get the current parameter value
                current_unit = rospy.get_param('/distance_unit')
                rospy.loginfo(f"Current unit: {current_unit}")
            else:
                rospy.logwarn("Parameter '/distance_unit' not set. Setting default to 'meters'.")
                rospy.set_param('/distance_unit', 'meters')
                current_unit = 'meters'

            # Switch between meters and feet based on the current unit
            if current_unit == 'meters':
                rospy.set_param('/distance_unit', 'feet')
                rospy.loginfo("Set parameter to feet")
            else:
                rospy.set_param('/distance_unit', 'meters')
                rospy.loginfo("Set parameter to meters")

            # Sleep for 5 seconds
            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('set_parameter_node', anonymous=True)
        
        # Create an instance of the SetParameterNode class
        set_param_node = SetParameterNode()
        
        # Run the node
        set_param_node.run()

    except rospy.ROSInterruptException:
        pass
