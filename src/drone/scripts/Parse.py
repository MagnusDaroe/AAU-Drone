#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil
import serial

class MavlinkReaderNode(Node):

    def __init__(self):
        super().__init__('mavlink_reader_node')
        self.publisher_ = self.create_publisher(String, 'mavlink_messages', 10)
        self.serial_port = '/dev/ttyTHS1'
        self.baud_rate = 57600

        self.timer = self.create_timer(0.1, self.timer_callback)

        try:
            self.the_connection = mavutil.mavlink_connection(self.serial_port, baud=self.baud_rate)
            self.the_connection.wait_heartbeat()
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
            raise

    def timer_callback(self):
        try:
            msg = self.the_connection.recv_match(blocking=True)
            raw_message = self.the_connection.mav.serial.read(self.the_connection.mav.bytes_available())
            if msg:
                mavlink_msg = String()
                mavlink_msg.data = str(msg)
                self.publisher_.publish(mavlink_msg)
                self.get_logger().info(f"Published MAVLink message: {mavlink_msg.data}")
                self.get_logger().info(f"Raw MAVLink data: {raw_message.hex()}")
        except Exception as e:
            self.get_logger().error(f"Failed to receive or publish MAVLink message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
