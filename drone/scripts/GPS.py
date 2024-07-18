#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(String, 'esp32_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.serial_port = '/dev/ttyUSB0'  # Adjust to your serial port
        self.baud_rate = 115200  # Adjust to your baud rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        
        self.get_logger().info(f'SerialReader node started, reading from {self.serial_port} at {self.baud_rate} baud rate.')

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Read from serial: {line}')
                msg = String()
                msg.data = line
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial port: {e}')

def main(args=None):
    rclpy.init(args=args)
    serial_reader = SerialReader()
    rclpy.spin(serial_reader)
    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
