#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from pymavlink import mavutil

class MavlinkReaderNode(Node):

    def __init__(self):
        super().__init__('mavlink_reader_node')
        self.publisher_ = self.create_publisher(String, 'mavlink_messages', 10)
        self.serial_port = '/dev/ttyTHS1'
        self.baud_rate = 115200


        self.timer = self.create_timer(0.1, self.timer_callback)

        self.the_connection = mavutil.mavlink_connection(self.USB_PORT,baud=self.BAUDRATE)
        self.the_connection.wait_heartbeat()
        self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")

    def timer_callback(self):
        msg = self.mavlink_connection.recv_match(blocking=True)
        if msg:
            mavlink_msg = String()
            mavlink_msg.data = str(msg)
            self.publisher_.publish(mavlink_msg)
            self.get_logger().info(f"Published MAVLink message: {mavlink_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
