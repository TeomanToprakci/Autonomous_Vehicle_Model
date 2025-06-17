#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    
    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        
        # TF Publisher Node
        Node(
            package='robot_control',
            executable='tf_publisher',
            name='tf_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # Motor Controller Serial Node
        Node(
            package='robot_control',
            executable='motor_controller_serial',
            name='motor_controller_serial',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'serial_port': serial_port,
                'baud_rate': 115200,
                'wheel_base': 0.3,
                'max_speed': 255
            }]
        ),
        
        # Button Navigation Node
        Node(
            package='robot_control',
            executable='button_navigation_node',
            name='button_navigation_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])

