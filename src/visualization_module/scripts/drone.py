#!/usr/bin/env python3
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, Twist, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Path
import numpy as np
import yaml
import time 
class StatePublisher(Node):

    def __init__(self):
        
        self.x=2.4
        self.y=2.00
        self.z=0.0
        self.q=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.th0=0.0
        self.hz=50
        self.linear_x=0
        self.angular_z=0
        

        super().__init__('statepublisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile)
        self.publish_transforms = self.create_publisher(TransformStamped,'/transformed_stamped', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.get_state = self.create_subscription(PoseStamped, '/pose',self.get_state,10)
        
        #self.nodeName = self.get_name()
        #self.get_logger().info("{0} started".format(self.nodeName))
        self.publishing_timer = self.create_timer(1.0 / self.hz, self.publisher_loop)  # Change 100 to your desired frequency (Hz)    


        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'map'
        self.odom_trans.child_frame_id = 'base_link'
        self.joint_state = JointState()

    def integrator(self, msg):
        linear = msg.linear
        angular = msg.angular

        # Log the data using ROS 2 Python logger
        #self.get_logger().info("Linear: [x:"+str(msg.linear.x)+", y: "+str(msg.linear.y)+", z: "+str(msg.linear.z)+"], Angular: [x: "+str(msg.angular.x)+", y:"+str(msg.angular.y)+" , z: "+str(msg.angular.z)+"]")
        self.linear_x=msg.linear.x
        self.angular_z=msg.angular.z

    def publisher_loop(self):
        
        now = self.get_clock().now()

        # (moving in a circle with radius=2)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = self.x
        self.odom_trans.transform.translation.y = self.y
        self.odom_trans.transform.translation.z = self.z
        self.odom_trans.transform.rotation = self.q
 
        
    
        self.publish_transforms.publish(self.odom_trans)
        self.broadcaster.sendTransform(self.odom_trans)
        
    def get_state(self,msg):
        self.x=msg.pose.position.x
        self.y=msg.pose.position.y
        self.z= msg.pose.position.z
        self.q=msg.pose.orientation
        

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = StatePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()