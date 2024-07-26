#!/usr/bin/env python3
from drone_interfaces.msg import DroneCommand
from geometry_msgs.msg import Quaternion, Twist, PoseStamped    
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String

class PIDController:
    def __init__(self):
        self.kp = 0.9
        self.ki = 0.0
        self.kd = 0.9

        self.integral_x = 0
        self.integral_y = 0
        self.integral_z = 0
        self.integral_yaw = 0

        self.prev_error_x = 0
        self.prev_error_y = 0
        self.prev_error_z = 0
        self.prev_error_yaw = 0

        self.prev_derivative_x = 0
        self.prev_derivative_y = 0
        self.prev_derivative_z = 0
        self.prev_derivative_yaw = 0

        self.alpha = 0.8

    def pid(self, x, y, z, yaw):
        error_x = x
        error_y = y
        error_z = z
        error_yaw = yaw

        self.integral_x += error_x
        self.integral_y += error_y
        self.integral_z += error_z
        self.integral_yaw += error_yaw

        raw_derivative_x = error_x - self.prev_error_x
        raw_derivative_y = error_y - self.prev_error_y
        raw_derivative_z = error_z - self.prev_error_z
        raw_derivative_yaw = error_yaw - self.prev_error_yaw

        derivative_x = self.alpha * self.prev_derivative_x + (1 - self.alpha) * raw_derivative_x
        derivative_y = self.alpha * self.prev_derivative_y + (1 - self.alpha) * raw_derivative_y
        derivative_z = self.alpha * self.prev_derivative_z + (1 - self.alpha) * raw_derivative_z
        derivative_yaw = self.alpha * self.prev_derivative_yaw + (1 - self.alpha) * raw_derivative_yaw

        control_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        control_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        control_z = self.kp * error_z + self.ki * self.integral_z + self.kd * derivative_z
        control_yaw = self.kp * error_yaw + self.ki * self.integral_yaw + self.kd * derivative_yaw

        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_z = error_z
        self.prev_error_yaw = error_yaw

        self.prev_derivative_x = derivative_x
        self.prev_derivative_y = derivative_y
        self.prev_derivative_z = derivative_z
        self.prev_derivative_yaw = derivative_yaw

        return control_x, control_y, control_z, control_yaw



class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(DroneCommand, '/cmd_fc', 10)   
        self.subscriber_state = self.create_subscription(PoseStamped, '/pose', self.get_state, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.data=0 
        self.x=0
        self.y=0
        self.z=0
        self.q=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pid=PIDController()
        self.m=0.5

        

    def timer_callback(self):
        if self.data==1:
            x_goal= 0.
            y_goal= 0.0
            z_goal = 10.
            yaw_goal = 0.


            error_x= x_goal - self.x
            error_y= y_goal - self.y
            error_z= z_goal - self.z
            error_yaw= yaw_goal - self.quaternion_to_euler(self.q.x, self.q.y, self.q.z, self.q.w)[2]

            control_x, control_y, control_z, control_yaw = self.pid.pid(error_x, error_y, error_z, error_yaw)


            msg = DroneCommand()
            
            max_thrust = 9.82*self.m*1.7
            min_thrust = -9.82*self.m*0.7
            desired_thrust = max(control_z, max_thrust * self.m)
            desired_thrust = min(desired_thrust, min_thrust * self.m)
            msg.cmd_thrust = control_z + 9.81*self.m
            msg.cmd_roll = (control_x)  # Set the desired roll value
            msg.cmd_pitch =   (control_y)  # Set the desired pitch value
            msg.cmd_yaw = (control_yaw)  # Set the desired yaw value
        


            self.publisher_.publish(msg)


    def get_state(self,msg):
        self.data=1
        self.x=msg.pose.position.x
        self.y=msg.pose.position.y
        self.z=msg.pose.position.z
        self.q=msg.pose.orientation

        


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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()