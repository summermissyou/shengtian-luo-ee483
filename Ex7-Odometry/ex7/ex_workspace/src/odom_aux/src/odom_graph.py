#!/usr/bin/env python3
import rospy
from odom_aux.msg import Pose2D
import matplotlib

class OdomGraph:
    def __init__(self):
        self.x_list = list()
        self.y_list = list()
    
    def pose_cb(self,msg):
        self.x_list.append(msg.x)
        self.y_list.append(msg.y)


if __name__ == '__main__':
    try:
        rospy.init_node('odom_graph', anonymous=True)
        output_to_file = False
        if rospy.has_param('/output_to_file'):
            rospy.logwarn("Has output to file")
            if rospy.get_param('/output_to_file') == True or rospy.get_param('/output_to_file') == "true":
                output_to_file=True
        if rospy.has_param('/only_output_to_file'):
            rospy.logwarn("Has only output to file")
            if rospy.get_param('/only_output_to_file') == True or rospy.get_param('/only_output_to_file') == "true":
                rospy.logwarn("only outputting to PDF!")
                output_to_file=True
                matplotlib.use("pdf")
        folder = "."
        if rospy.has_param('output_folder'):
            folder = rospy.get_param('output_folder')
                                
        import matplotlib.pyplot as plt
        og = OdomGraph()
        rospy.Subscriber("pose", Pose2D, og.pose_cb)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            plt.plot(og.x_list, og.y_list,'bo-',)
            plt.axis([-0.5,5,-0.5,3])
            plt.xlabel('x (m)')
            plt.ylabel('y (m)')
            plt.title('Vehicle Odometry')
            if output_to_file:
                plt.savefig(folder + "/output_plot.png")
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
