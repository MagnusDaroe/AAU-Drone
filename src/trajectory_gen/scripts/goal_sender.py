#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('goal_sender')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_publisher_ = self.create_publisher(Marker, '/visualization_marker', 10)  # Publisher for the marker
        self.subscription = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.poses = []
        self.poses.append([0.0, 0.0, 10.0])
        self.poses.append([0.0, 10.0, 10.0])
        self.poses.append([10.0, 10.0, 10.0])
        self.poses.append([10.0, 0.0, 10.0])
        self.current_pose_index = 0
        self.current_position = [0.0, 0.0, 0.0]

    def distance(self, pos1, pos2):
        return np.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)

    def update_position(self, new_position):
        self.current_position = new_position
        if self.distance(self.current_position, self.poses[self.current_pose_index]) <= 0.3:
            self.current_pose_index = (self.current_pose_index + 1) % len(self.poses)

    def pose_callback(self, msg):
        self.current_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.update_position(self.current_position)

    def timer_callback(self):
        current_target_pose = self.poses[self.current_pose_index]
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = "map"
        pose_stamped_msg.pose.position.x = current_target_pose[0]
        pose_stamped_msg.pose.position.y = current_target_pose[1]
        pose_stamped_msg.pose.position.z = current_target_pose[2]
        pose_stamped_msg.pose.orientation.x = 0.0
        pose_stamped_msg.pose.orientation.y = 0.0
        pose_stamped_msg.pose.orientation.z = 0.0
        pose_stamped_msg.pose.orientation.w = 1.0
        self.publisher_.publish(pose_stamped_msg)
        
        self.publish_ball_marker(current_target_pose)

    def publish_ball_marker(self,current_target_pose):
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.ns = "ball_marker"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = current_target_pose[0]
        marker_msg.pose.position.y = current_target_pose[1]
        marker_msg.pose.position.z = current_target_pose[2]
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.4  # Diameter in x direction (radius * 2)
        marker_msg.scale.y = 0.4  # Diameter in y direction (radius * 2)
        marker_msg.scale.z = 0.4  # Diameter in z direction (radius * 2)
        marker_msg.color.a = 1.0  # Alpha (transparency)
        marker_msg.color.r = 0.0  # Red
        marker_msg.color.g = 1.0  # Green
        marker_msg.color.b = 0.0  # Blue
        self.marker_publisher_.publish(marker_msg)


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
