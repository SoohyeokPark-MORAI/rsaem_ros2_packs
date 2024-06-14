# rsaembot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    
        Node(
        package='rsaem_node', # Package Name
        executable='rsaembot_core', # Executable file
        output='screen',
        emulate_tty=True),
    ])
