#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Create the launch description

    # Declare the launch arguments and set their default values
    use_python_arg = DeclareLaunchArgument(
        'use_python',
        default_value='true',
        description='Whether to use Python or C++ nodes'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
        description='Wheel radius'
    )

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
        description='Wheel separation'
    )
    # read runtime values of the launch arguments
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')

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

    simple_controller_py = Node(
        package='bumperbot_controller',
        executable='simple_controller.py',
        parameters=[
            {'wheel_radius': wheel_radius},
            {'wheel_separation': wheel_separation},
        ],
        condition=IfCondition(use_python)
    )

    simple_controller_cpp = Node(
        package='bumperbot_controller',
        executable='simple_controller',
        parameters=[
            {'wheel_radius': wheel_radius},
            {'wheel_separation': wheel_separation},
        ],
        condition=UnlessCondition(use_python)
    )

    # Return the launch description object
    return LaunchDescription(
        [
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            simple_vel_controller_spawner,
            simple_controller_py,
            simple_controller_cpp,
        ]
    )
