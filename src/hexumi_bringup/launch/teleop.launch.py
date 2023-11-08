import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.1}
            ]
    ) 
    ld.add_action(joy_node)
    
    teleop_joy = Node(
        package='hexumi_teleop_joy',
        executable='hexumi_teleop_joy',
    )
    ld.add_action(teleop_joy)    
    
    return ld