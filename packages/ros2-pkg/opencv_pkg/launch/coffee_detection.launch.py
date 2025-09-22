#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Coffee menu detection node
        Node(
            package='opencv_ros',
            executable='coffee_menu',
            name='coffee_menu_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        # Coffee sticker detection node
        Node(
            package='opencv_ros',
            executable='coffee_sticker',
            name='coffee_sticker_node',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        
        Node(
            package='mission_ctrl',
            executable='mission2',
            name='mission2_node',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])
