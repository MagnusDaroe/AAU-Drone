#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import threading
import time
from drone_interfaces.msg import DroneCommand, DroneStatus, DroneIMU
from drone_interfaces.srv import Clock
import numpy as np

class FC_Commander(Node):
    # Class constructor
    def __init__(self):
        super().__init__('fc_command_listener')

        # Fc command variables
        self.USB_PORT = '/dev/ttyTHS1'
        self.BAUDRATE = 57600

        # IMU variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.publisher_imu = self.create_publisher(
            DroneIMU,
            '/imu_fc',
            10
        )
        # Set the logging level based on command-line argument or default to INFO
        self.log_level = self.get_logger().get_effective_level()

        # Connect to the flight controller
        self.drone_init()

    def Publish_IMU_data(self):
        """
        Publish the IMU data\n
        msg: DroneIMU message - {xxx}
        """
        while rclpy.ok():
            self.get_IMU_data()
           
            request_time = time.time()
            self.get_logger().info("Requesting IMU data")
            
            # Request the imu data
            self.the_connection.mav.request_data_stream_send(
                self.the_connection.target_system,
                self.the_connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                1,
                1
            )

            # Wait for the battery voltage
            drone = self.the_connection.recv_match(type='ATTITUDE', blocking=True)

            self.get_logger().info(f"Took {time.time() - request_time} seconds to receive the IMU data")

            # Receive the IMU data
            self.get_logger().info(f"Received IMU data: Roll={drone.roll}, Pitch={drone.pitch}, Yaw={drone.yaw}")

            # Publish the data. create a DroneStatus msg object
            msg = DroneIMU()

            # Get the IMU data
            msg.timestamp = time.time()
            msg.roll = float(drone.roll)
            msg.pitch = float(drone.pitch)
            msg.yaw = float(drone.yaw)

            # Publish the message
            self.publisher_imu.publish(msg)


    # Drone functions
    def drone_init(self):
        """
        Start the connection to the flight controller
        """
        self.get_logger().info("Connecting to MAVLink...")
        self.the_connection = mavutil.mavlink_connection(self.USB_PORT,baud=self.BAUDRATE)
        self.the_connection.wait_heartbeat()
        self.get_logger().info("Connected to MAVLink.")
        time.sleep(2)


def main(args=None):
    # Initialize the node
    rclpy.init(args=args)
    
    # Create the node - daemon=True to stop the thread when the main thread stops
    FC_node = FC_Commander()
    threading.Thread(target=FC_node.Publish_IMU_data, daemon=True).start()

    # Spin the node
    rclpy.spin(FC_node)

    # Destroy the node when code is stopped
    FC_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





