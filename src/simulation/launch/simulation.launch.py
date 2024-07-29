import os

from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():




    package_name='simulation' 
    pkg_path = os.path.join(get_package_share_directory(package_name))

    
    node_robot_state_publisher = Node(
        package='simulation',
        executable='simulation',
    )

    return LaunchDescription([

    node_robot_state_publisher,
    ])