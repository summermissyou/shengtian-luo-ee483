#!/usr/bin/env python3
## ORIGINAL CODE FROM EECE5560 UMASS Lowel Class by Professor Robinette
## MODIFIED FROM EECE5560: Changed to Duckiebot dynamics
import rospy
from numpy import arange,sign,cos,sin
from random import random
from std_msgs.msg import Float32

class VehicleDynamics:
    def __init__(self):
        self.x = 0 # initial position, m
        self.y = 0
        self.omega = 0
        self.theta = 1.57
        self.v = 0.3
        self.control = 0 # control input, m/s^2
        self.L = 0.05 # half of body lenght in between wheels in meters
        self.R = 0.0318 # radius of the wheel in meters
        self.k = 27 # Motor gain
        self.gain = 1 # MM motor gain parameter
        self.trim = 0 # MM tuning motor skew
        self.omega_max = 8
        self.v_max = 1
        self.limit = 1
        self.noise_mag = 0.01
            
    def iterate_mm(self, dt):
        # a = engine acceleration - friction - drag
        self.omega = self.bound(self.omega,-self.omega_max, self.omega_max)
        self.x = self.x+self.v*cos(self.theta)*dt
        self.y = self.y+self.v*sin(self.theta)*dt
        self.theta = self.theta+self.omega*dt + self.noise_mag*random()
        return self.x, self.y, self.theta
    
    # def iterate_mm2(self, dt):
    #     # print(self.v,self.omega)
    #     self.omega = self.bound(self.omega,-self.omega_max, self.omega_max)
    #     k_r = k_l = self.k

    #     # adjusting k by gain and trim
    #     k_r_inv = (self.gain + self.trim) / k_r
    #     k_l_inv = (self.gain - self.trim) / k_l
        
    #     omega_r = (self.v + 0.5 * self.omega * self.L*2) / self.R
    #     omega_l = (self.v - 0.5 * self.omega * self.L*2) / self.R
    #     # conversion from motor rotation rate to duty cycle
    #     # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
    #     u_r = omega_r * k_r_inv
    #     # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
    #     u_l = omega_l * k_l_inv
        
    #     # limiting output to limit, which is 1.0 for the duckiebot
    #     v_r = self.bound(u_r, -self.limit, self.limit)
    #     v_l = self.bound(u_l, -self.limit, self.limit)

    #     omega_r = v_r/ k_r_inv
    #     omega_l = v_l / k_l_inv

    #     # Compute linear and angular velocity of the platform
    #     v = (self.R * omega_r + self.R * omega_l) / 2.0
    #     omega = (self.R * omega_r - self.R * omega_l) / (2*self.L)
    #     # It gives the same values as input
    #     # v = (v_r + v_l) / 2
    #     # omega = (v_r - v_l) / (self.L)

    #     # print(v,omega)
    #     self.x = self.x+v*cos(self.theta)*dt
    #     self.y = self.y+v*sin(self.theta)*dt
    #     self.theta = self.theta+omega*dt
    #     return self.x, self.y, self.theta
    
    def update_control_mm(self, control):
        self.omega = control.data
    def update_control(self, control):
        self.control = control.data

    def bound(self,value, low, high):
        #FROM dt-interface-car duckiebot git repo: https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/kinematics_node.py
        """
        Trims a value to be between some bounds.

        Args:
            value: the value to be trimmed
            low: the minimum bound
            high: the maximum bound

        Returns:
            the trimmed value
        """
        return max(min(value, high), low)

if __name__ == '__main__':
    try:
        desired = 0
        time_step = 0.1
        rospy.init_node('vehicle_dynamics')
        vd = VehicleDynamics()
        rospy.Subscriber("control_input", Float32, vd.update_control_mm)
        pub_x = rospy.Publisher("position", Float32, queue_size=10)
        pub_d = rospy.Publisher("desired", Float32, queue_size=10)
        pub_error = rospy.Publisher("error", Float32, queue_size=10)
        # pub_xd = rospy.Publisher("velocity", Float32, queue_size=10)
        pub_y = rospy.Publisher("position_y", Float32, queue_size=10)
        pub_theta = rospy.Publisher("position_theta", Float32, queue_size=10)
        # pub_error = rospy.Publisher("error", Float32, queue_size=10)
        
        rate = rospy.Rate(0.5)
        # wait until param says controller is ready
        while not rospy.is_shutdown():
            if rospy.has_param("controller_ready"):
                if rospy.get_param("controller_ready") == "true":
                    break
            rospy.logwarn("Waiting for controller_ready to be true")
            rate.sleep()
        
        # wait until param says graph is ready
        while not rospy.is_shutdown():
            if rospy.has_param("graph_ready"):
                if rospy.get_param("graph_ready") == "true":
                    break
            rospy.logwarn("Waiting for graph_ready to be true")
            rate.sleep()
        
        rospy.logwarn("Starting dynamics")
        rate = rospy.Rate(1.0/time_step)
        # run vehicle dynamics for ~10 seconds
        start_time = rospy.get_time()
        updated_desired = False
        # while not rospy.is_shutdown():
        #     time_elapsed = rospy.get_time() - start_time
        #     if not updated_desired and time_elapsed > 15:
        #         updated_desired = True
        #         desired += 30
                
        #     # quit after 30 sec of running
        #     if time_elapsed > 30:
        #         rospy.set_param("controller_ready", "false")
        #         rospy.logwarn("Time limit reached -- stopping dynamics")
        #         exit()
        #     vd.iterate(time_step)
        #     pub_xd.publish(vd.xd)
        #     pub_x.publish(vd.x)
        #     pub_d.publish(desired)
        #     pub_error.publish(desired - vd.x)
        #     #rospy.logwarn("v=%f, x=%f, e=%f" % (vd.xd, vd.x, desired-vd.x))
        #     rate.sleep()
        while not rospy.is_shutdown():
            time_elapsed = rospy.get_time() - start_time
            if not updated_desired and time_elapsed > 15:
                updated_desired = True
                desired += 0.8
                
            # quit after 30 sec of running
            if time_elapsed > 30:
                rospy.set_param("controller_ready", "false")
                rospy.logwarn("Time limit reached -- stopping dynamics")
                exit()
            vd.iterate_mm(time_step)
            pub_x.publish(vd.x)
            pub_y.publish(vd.y)
            pub_theta.publish(vd.theta)
            pub_d.publish(desired)
            pub_error.publish(desired - vd.theta)
            #rospy.logwarn("v=%f, x=%f, e=%f" % (vd.xd, vd.x, desired-vd.x))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
