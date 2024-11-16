#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import matplotlib

class ControlsGraph:
    def __init__(self):
        self.xp_list = list()
        self.yp_list = list()
        self.xpy_list = list()
        self.ypy_list = list()
        self.xpt_list = list()
        self.ypt_list = list()
        self.xv_list = list()
        self.yv_list = list()
        self.xd_list = list()
        self.yd_list = list()
        self.start_time = 0
        rospy.Subscriber("position", Float32, self.pos_cb)
        rospy.Subscriber("position_y", Float32, self.posy_cb)
        rospy.Subscriber("position_theta", Float32, self.postheta_cb)
        rospy.Subscriber("desired", Float32, self.desired_cb)
    def start_clock(self):
        self.start_time = rospy.get_time()
    
    def pos_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xp_list.append(rospy.get_time()-self.start_time)
        self.yp_list.append(msg.data)
    def posy_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xpy_list.append(rospy.get_time()-self.start_time)
        self.ypy_list.append(msg.data)
    def postheta_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xpt_list.append(rospy.get_time()-self.start_time)
        self.ypt_list.append(msg.data)
        
    def vel_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xv_list.append(rospy.get_time()-self.start_time)
        self.yv_list.append(msg.data)
        
    def desired_cb(self,msg):
        if self.start_time == 0:
            self.start_clock()
        self.xd_list.append(rospy.get_time()-self.start_time)
        self.yd_list.append(msg.data)




if __name__ == '__main__':
    try:
        rospy.init_node('controls_graph', anonymous=True)
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
        cg = ControlsGraph()
        rospy.set_param("graph_ready","true")
        rate = rospy.Rate(5) # 5hz
        # fig1, ax1 = plt.subplots()
        while not rospy.is_shutdown():   

            # ax1.plot(cg.xpt_list, cg.ypt_list, 'b-')
            # ax1.plot(cg.xd_list, cg.yd_list, 'g-')
            # ax1.set_xlabel('Time (s)')
            # ax1.set_ylabel('Measured Value')
            # ax1.set_title('Orientation Motion')
            # ax1.set_legend(('$theta$ (rad)', '$theta_d$ (rad)'))
            plt.figure(0)
            plt.plot(cg.xpt_list, cg.ypt_list, 'b-')
            plt.plot(cg.xd_list, cg.yd_list, 'g-')
            plt.xlabel('Time (s)')
            plt.ylabel('Measured Value')
            plt.title('Orientation Motion')
            plt.legend(('$theta$ (rad)', '$theta_d$ (rad)'))
            plt.figure(1)
            plt.plot(cg.yp_list, cg.ypy_list, 'b-')
            plt.xlabel('MM x position')
            plt.ylabel('MM y position')
            plt.title('Position of the MM')
            # plt.legend(('$theta$ (rad)', '$theta_d$ (rad)'))
            if output_to_file:
                plt.savefig(folder + "/output_plot.png")
            plt.pause(0.05)        
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


# fig1, ax1 = plt.subplots()
# ax1.plot(x, y)
# ax1.set_title("Axis 1 title")
# ax1.set_xlabel("X-label for axis 1")

# z = np.sin(x)
# fig2, (ax2, ax3) = plt.subplots(nrows=2, ncols=1) # two axes on figure
# ax2.plot(x, z)
# ax3.plot(x, -z)

# w = np.cos(x)
# ax1.plot(x, w) # can continue plotting on the first axis
