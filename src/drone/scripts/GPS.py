#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from drone.msg import GPSData
import math

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher_ = self.create_publisher(GPSData, 'esp32_data', 10)
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
                
                # Parse the GPS data from the string
                if line.startswith('<gps>') and line.endswith('</gps>'):
                    data = self.parse_gps_data(line)
                    if data:
                        msg = GPSData()
                        msg.lat = data.get('lat', float('nan'))  # Use NaN for missing latitude
                        msg.lon = data.get('lon', float('nan'))  # Use NaN for missing longitude
                        msg.date = data.get('date', 'N/A')  # Use 'N/A' for missing date
                        msg.time = data.get('time', 'N/A')  # Use 'N/A' for missing time
                        msg.course = data.get('course', float('nan'))  # Use NaN for missing course
                        msg.speed = data.get('speed', float('nan'))  # Use NaN for missing speed
                        msg.pdop = data.get('pdop', float('nan'))  # Use NaN for missing pdop
                        msg.hdop = data.get('hdop', float('nan'))  # Use NaN for missing hdop
                        self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial port: {e}')

    def parse_gps_data(self, data):
        # Simple XML parsing
        try:
            gps_data = {}
            
            if '<lat>' in data and '</lat>' in data:
                gps_data['lat'] = float(data.split('<lat>')[1].split('</lat>')[0])
            else:
                gps_data['lat'] = float('nan')
            
            if '<lon>' in data and '</lon>' in data:
                gps_data['lon'] = float(data.split('<lon>')[1].split('</lon>')[0])
            else:
                gps_data['lon'] = float('nan')
            
            if '<date>' in data and '</date>' in data:
                gps_data['date'] = data.split('<date>')[1].split('</date>')[0]
            else:
                gps_data['date'] = 'N/A'
            
            if '<time>' in data and '</time>' in data:
                gps_data['time'] = data.split('<time>')[1].split('</time>')[0]
            else:
                gps_data['time'] = 'N/A'
            
            if '<course>' in data and '</course>' in data:
                gps_data['course'] = float(data.split('<course>')[1].split('</course>')[0])
            else:
                gps_data['course'] = float('nan')
            
            if '<speed>' in data and '</speed>' in data:
                gps_data['speed'] = float(data.split('<speed>')[1].split('</speed>')[0])
            else:
                gps_data['speed'] = float('nan')
            
            if '<pdop>' in data and '</pdop>' in data:
                gps_data['pdop'] = float(data.split('<pdop>')[1].split('</pdop>')[0])
            else:
                gps_data['pdop'] = float('nan')
            
            if '<hdop>' in data and '</hdop>' in data:
                gps_data['hdop'] = float(data.split('<hdop>')[1].split('</hdop>')[0])
            else:
                gps_data['hdop'] = float('nan')
            
            return gps_data
        except Exception as e:
            self.get_logger().error(f'Error parsing GPS data: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    serial_reader = SerialReader()
    rclpy.spin(serial_reader)
    serial_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
