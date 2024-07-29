#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from std_msgs.msg import String
from drone_interfaces.msg import DroneCommand
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('simulation')
        self.publisher_ = self.create_publisher(PoseStamped, '/pose', 10)
        self.subcriber_pid=self.create_subscription(DroneCommand,'/cmd_fc',self.pid_callback,10)
        self.hz = 10
        self.timer = self.create_timer(1/self.hz, self.timer_callback)
        
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.z_dot = 0.0
        self.q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.m = 0.5
        
        self.T = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


    def pid_callback(self,msg):
        self.T = msg.cmd_thrust
        self.roll = msg.cmd_roll
        self.pitch = msg.cmd_pitch
        self.yaw = msg.cmd_yaw
    def timer_callback(self):
        self.yaw= -np.pi/2
        # Update velocities based on the translational dynamics
        # Final linearized form
        ax = (self.T / self.m) * self.roll  # Roll goes to forward (x)
        ay = (self.T / self.m) * self.pitch  # Pitch goes to side (y)
        az = (self.T / self.m) - 9.81  # Vertical (z)
        

        
        self.x_dot += ax * (1 / self.hz)
        self.y_dot += ay * (1 / self.hz)
        self.z_dot += az * (1 / self.hz)
        
    # Limit velocities to max velocity of 1 m/s
        max_vel = 3.0
        self.x_dot = np.clip(self.x_dot, -max_vel, max_vel)
        self.y_dot = np.clip(self.y_dot, -max_vel, max_vel)
        self.z_dot = np.clip(self.z_dot, -9.82, max_vel)

        # Update positions based on velocities
        self.x0 += self.x_dot * (1 / self.hz)
        self.y0 += self.y_dot * (1 / self.hz)
        self.z0 += self.z_dot * (1 / self.hz)
        
        if self.z0 < 0:
            self.z0 = 0.0
            self.z_dot = 0.0
        
        # Publish the updated pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.x0
        msg.pose.position.y = self.y0
        msg.pose.position.z = self.z0
        msg.pose.orientation = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        self.publisher_.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()



