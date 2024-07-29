#!/usr/bin/env python3
from drone_interfaces.msg import DroneCommand
from geometry_msgs.msg import Quaternion, PoseStamped    
import rclpy
from rclpy.node import Node
import numpy as np


class PIDController:
    def __init__(self, 
                 kp_xy=10, ki_xy=0.0, kd_xy=5, 
                 kp_z=10.0, ki_z=0.0, kd_z=5,
                 kp_yaw=1.0, ki_yaw=0.0, kd_yaw=0.5, 
                 alpha=0.8, integral_limit=100.0, derivative_smoothing_factor=0.1):
        self.kp_xy = kp_xy
        self.ki_xy = ki_xy
        self.kd_xy = kd_xy
        self.kp_z = kp_z
        self.ki_z = ki_z
        self.kd_z = kd_z
        self.kp_yaw = kp_yaw
        self.ki_yaw = ki_yaw
        self.kd_yaw = kd_yaw
        self.alpha = alpha
        self.integral_limit = integral_limit
        self.derivative_smoothing_factor = derivative_smoothing_factor
        
        # Initialize PID terms for x and y to use the same kp, ki, kd values
        self.kp_x = self.kp_xy
        self.ki_x = self.ki_xy
        self.kd_x = self.kd_xy
        self.kp_y = self.kp_xy
        self.ki_y = self.ki_xy
        self.kd_y = self.kd_xy

        self.alpha = alpha
        self.integral_limit = integral_limit
        self.derivative_smoothing_factor = derivative_smoothing_factor

        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.integral_yaw = 0.0

        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.prev_error_yaw = 0.0

        self.prev_derivative_x = 0.0
        self.prev_derivative_y = 0.0
        self.prev_derivative_z = 0.0
        self.prev_derivative_yaw = 0.0

        self.smoothed_derivative_x = 0.0
        self.smoothed_derivative_y = 0.0
        self.smoothed_derivative_z = 0.0
        self.smoothed_derivative_yaw = 0.0

    def pid(self, x, y, z, yaw):
        error_x = x
        error_y = y
        error_z = z
        error_yaw = yaw

        # Integral calculation with windup guard
        self.integral_x += error_x
        self.integral_y += error_y
        self.integral_z += error_z
        self.integral_yaw += error_yaw

        self.integral_x = max(min(self.integral_x, self.integral_limit), -self.integral_limit)
        self.integral_y = max(min(self.integral_y, self.integral_limit), -self.integral_limit)
        self.integral_z = max(min(self.integral_z, self.integral_limit), -self.integral_limit)
        self.integral_yaw = max(min(self.integral_yaw, self.integral_limit), -self.integral_limit)

        # Derivative calculation
        raw_derivative_x = error_x - self.prev_error_x
        raw_derivative_y = error_y - self.prev_error_y
        raw_derivative_z = error_z - self.prev_error_z
        raw_derivative_yaw = error_yaw - self.prev_error_yaw

        # Apply exponential moving average for smoothing
        self.smoothed_derivative_x = (self.derivative_smoothing_factor * raw_derivative_x +
                                      (1 - self.derivative_smoothing_factor) * self.smoothed_derivative_x)
        self.smoothed_derivative_y = (self.derivative_smoothing_factor * raw_derivative_y +
                                      (1 - self.derivative_smoothing_factor) * self.smoothed_derivative_y)
        self.smoothed_derivative_z = (self.derivative_smoothing_factor * raw_derivative_z +
                                      (1 - self.derivative_smoothing_factor) * self.smoothed_derivative_z)
        self.smoothed_derivative_yaw = (self.derivative_smoothing_factor * raw_derivative_yaw +
                                        (1 - self.derivative_smoothing_factor) * self.smoothed_derivative_yaw)

        # PID control
        control_x = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * self.smoothed_derivative_x
        control_y = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * self.smoothed_derivative_y
        control_z = self.kp_z * error_z + self.ki_z * self.integral_z + self.kd_z * self.smoothed_derivative_z
        control_yaw = self.kp_yaw * error_yaw + self.ki_yaw * self.integral_yaw + self.kd_yaw * self.smoothed_derivative_yaw

        # Save previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z
        self.prev_error_yaw = error_yaw

        return control_x, control_y, control_z, control_yaw*0



class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(DroneCommand, '/cmd_fc', 10)   
        self.subscriber_state = self.create_subscription(PoseStamped, '/pose', self.get_state, 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data = 0 
        self.x = 0
        self.y = 0
        self.z = 0
        self.q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pid = PIDController()
        self.m = 0.5
        self.x_goal = 0
        self.y_goal = 0
        self.z_goal = 0
        self.yaw_goal = 0
        self.subscriber = self.subscriber_state = self.create_subscription(PoseStamped, '/goal_pose', self.get_goalpose ,10)
    
    
    def get_goalpose(self,msg):    
        self.x_goal = msg.pose.position.x
        self.y_goal = msg.pose.position.y
        self.z_goal = msg.pose.position.z
        self.yaw_goal = self.quaternion_to_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)[2]
    def timer_callback(self):
        if self.data == 1:
 
            error_x = (self.x_goal- self.x)
            error_y = (self.y_goal - self.y)
            error_z = (self.z_goal- self.z)
            
   
            error_yaw = self.yaw_goal - self.quaternion_to_euler(self.q.x, self.q.y, self.q.z, self.q.w)[2]

            control_x, control_y, control_z, control_yaw = self.pid.pid(error_x, error_y, error_z, error_yaw)

            msg = DroneCommand()

            max_thrust = 9.82 * self.m * 1.7
            min_thrust = -9.82 * self.m * 0.7
            desired_thrust = np.clip(control_z + 9.81 * self.m, min_thrust, max_thrust)
            msg.cmd_thrust = desired_thrust

            max_angle = np.radians(30)
            msg.cmd_roll =  np.clip(control_x, -max_angle, max_angle)
            msg.cmd_pitch = np.clip(control_y, -max_angle, max_angle)
            msg.cmd_yaw = control_yaw
            print("x_error", error_x, "y_error", error_y, "z_error", error_z)
            print("roll", msg.cmd_roll, "pitch", msg.cmd_pitch, "thruster", msg.cmd_thrust) 

            self.publisher_.publish(msg)

    def get_state(self, msg):
        self.data = 1
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        self.q = msg.pose.orientation

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to euler angles
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
