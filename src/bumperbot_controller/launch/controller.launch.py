#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch description

    # start joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster',
                    '--controller-manager',
                      '/controller_manager' # set namespace
        ]
    )
    
    # start simple_velocity_controller
    simple_vel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['simple_velocity_controller',
                    '--controller-manager',
                      '/controller_manager' # set namespace
        ]
    )

    # Return the launch description object
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            simple_vel_controller_spawner,
        ]
    )
