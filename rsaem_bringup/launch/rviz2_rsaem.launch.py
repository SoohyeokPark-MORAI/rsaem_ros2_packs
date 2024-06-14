#!/usr/bin/env python3

# Copyright 2024 JetsonAI CO., LTD.
#
# Author: Kate Kim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('rsaem_description'),
        'rviz',
        'model.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])
