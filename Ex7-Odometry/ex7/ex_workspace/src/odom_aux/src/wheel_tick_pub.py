#!/usr/bin/env python3
import csv
import rospy
from odom_aux.msg import DistWheel

def pattern_generator_psu(i):
    if i < 2:
        return (0,0.0785) # left
    elif i >= 2 and i < 22:
        return (.1,.1) # up 
    elif i >= 22 and i < 24:
        return (0.0785,0) # right
    elif i >= 24 and i < 34:
        return (0.1,0.1) # space top for P 
    elif i >= 34 and i < 36:
        return (0.0785,0) # right
    elif i >= 36 and i < 46:
        return (.1,.1) # down of P
    elif i >= 47 and i < 49:
        return (0.0785,0) # right
    elif i >= 49 and i < 59:
        return (.1,.1) # bottom of P
    elif i >= 60 and i < 62:
        return (0,0.0785) # left
    elif i >= 62 and i < 71:
        return (.1,.1) # down finish P
    elif i >= 71 and i < 73:
        return (0,0.0785) # left to go to S
    elif i >= 73 and i < 98:
        return (.1,.1) # down S
    elif i >= 98 and i < 100:
        return (0,0.0785) # left
    elif i >= 100 and i < 109:
        return (.1,.1) # up bottom S
    elif i >= 109 and i < 111:
        return (0,0.0785) # left
    elif i >= 112 and i < 122:
        return (.1,.1) # middle of S    
    elif i >= 122 and i < 124:
        return (0.0785,0) # right
    elif i >= 124 and i < 134:
        return (.1,.1) # up second part of S
    elif i >= 134 and i < 136:
        return (0.0785,0) # # right for top of S
    elif i >= 136 and i < 150:
        return (.1,.1) # Top of S
    elif i >= 150 and i < 152:
        return (0.0785,0) # right for down U.
    elif i >= 152 and i < 172:
        return (.1,.1) # down U
    elif i >= 172 and i < 174:
        return (0,0.0785) # left for bottom U
    elif i >= 174 and i < 184:
        return (.1,.1) # bottom  U
    elif i >= 184 and i < 186:
        return (0,0.0785) # left for up U.
    elif i >= 186 and i < 206:
        return (.1,.1) # up second part of U
    return (0,0)



if __name__ == "__main__":
    rospy.init_node('wheel_tick_pub', anonymous=True)
    pub = rospy.Publisher("dist_wheel", DistWheel, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    for i in range(50):
        pub.publish(DistWheel(0,0))
        if rospy.is_shutdown():
            break
        rate.sleep()

    for i in range(210):
        ticks_left,ticks_right = pattern_generator_psu(i)
        rospy.logwarn("left: %f right: %f" % (ticks_left,ticks_right))
        ticks = DistWheel()
        ticks.dist_wheel_left = ticks_left
        ticks.dist_wheel_right = ticks_right
        pub.publish(ticks)
        if rospy.is_shutdown():
            break
        rate.sleep()
    
