#!/usr/bin/env python3

from std_msgs.msg import Float32
import rospy

class PIDController:
    def __init__(self, kp, ki, kd, setpoint, control_limit=5.0, integral_limit=None):
        rospy.set_param("controller_ready", "true")
        rospy.loginfo("Controller ready parameter set to true")

        self.error_sub = rospy.Subscriber('error', Float32, self.controller, queue_size=10)
        self.control_pub = rospy.Publisher('control_input', Float32, queue_size=10)
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0
        self.integral_limit = integral_limit
        self.control_limit = control_limit
        self.setpoint = setpoint
        self.previous_time = rospy.get_time()
    
    def controller(self, msg):
        # Calculate error
        error = self.setpoint - msg.data
        current_time = rospy.get_time()
        dt = current_time - self.previous_time

        if dt <= 0.001:  # Prevent division by zero and extremely small dt
            return

        # Proportional term
        p = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        i = self.ki * self.integral

        # Derivative term with smoothing
        derivative = (error - self.previous_error) / dt
        d = self.kd * derivative

        # Update previous error and time
        self.previous_error = error
        self.previous_time = current_time

        # PID output (control input)
        control_input = p + i + d

        # Clamp control input to prevent runaway or excessive control
        control_input = max(min(control_input, self.control_limit), -self.control_limit)
        rospy.loginfo("Published control input: %f", control_input)
        # Publish control input
        self.control_pub.publish(Float32(control_input))
        
        # Log control input for debugging
        rospy.loginfo("Error: %.4f, Control Input: %.4f", error, control_input)

if __name__ == "__main__":
    rospy.init_node("PIDController", anonymous=True)
    
    # Adjust gains based on response requirements
    kp = -1.8 # Reduce proportional for smoother response
    ki = -0.01  # Integral limited to avoid windup
    kd = 0.005  # Derivative to counter sudden shifts
    
    # Set desired orientation and limits
    setpoint = 0  # Desired final orientation in radians
    integral_limit = 3 # Anti-windup for integral part
    control_limit = 1 # Cotrol signal clamping

    controller = PIDController(kp, ki, kd, setpoint, control_limit, integral_limit)
    rospy.spin()  # Keep node alive
