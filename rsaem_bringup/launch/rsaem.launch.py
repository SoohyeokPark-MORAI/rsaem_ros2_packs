#!/usr/bin/env python3

# Copyright 2024 JetsonAI CO., LTD.
#
# Author: Kate Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    LIDAR_MODEL = os.environ['LIDAR_MODEL']
    #LIDAR_MODEL = 'LDS-01'
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    can_port = LaunchConfiguration('can_port', default='can0')

    rsaem_param_dir = LaunchConfiguration(
        'rsaem_param_dir',
        default=os.path.join(
            get_package_share_directory('rsaem_bringup'),
            'param',
            'rsaem.yaml'))

    if LIDAR_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LIDAR_MODEL == 'YDLIDAR':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ydlidar_launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'can_port',
            default_value=can_port,
            description='Connected CAN0'),

        DeclareLaunchArgument(
            'rsaem_param_dir',
            default_value=rsaem_param_dir,
            description='Full path to rsaembot parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/rsaem_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='rsaem_node',
            executable='rsaembot_core',
            parameters=[rsaem_param_dir],
            arguments=['-i', can_port],
            output='screen'),
    ])
