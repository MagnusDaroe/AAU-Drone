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




    package_name='vizualisation_module' 
    pkg_path = os.path.join(get_package_share_directory(package_name))


    #robot1 = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name),'launch','sim_robot1.launch.py')]))



    rvizfile=os.path.join(pkg_path,'config','rvizconfig.rviz')

    node_rviz2= Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rvizfile]
    )

    
    xacro_file = os.path.join(pkg_path,'models','robot1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
   
    

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': False}
    
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #output='screen',
        parameters=[params]
    )

    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['joint_states']}]
    )


    statepublish= Node(
        package='vizualisation_module',
        executable='drone.py',
        output='screen',
        
    )
    return LaunchDescription([
        node_rviz2,
    node_robot_state_publisher,
    joint_state_publisher,
    statepublish

    ])